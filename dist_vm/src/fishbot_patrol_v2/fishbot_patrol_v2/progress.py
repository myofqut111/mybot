"""巡检进度快照：把「跑到第几轮第几个点」落到磁盘，支持断电/重启后续跑。

同样**不依赖 ROS**，只处理纯数据与文件读写。

设计要点
--------
- 写入用「临时文件 + ``os.replace``」，进程被杀也不会留下半个 JSON。
- 快照带路线指纹（路线名 + 模式 + 轮数 + 点位组成），换了路线就自动作废，
  避免「拿 A 路线的进度去续 B 路线」这种脏数据。
- 快照里连「当前这一轮的点位顺序」一起存，RANDOM 模式洗过牌也能正确续跑。
"""

import hashlib
import json
import os
import tempfile
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import List, Optional, Sequence

# v2：指纹里加入点位组成。旧快照（v1）一律作废，宁可重跑一轮，
# 也不要按新路线的下标去解释旧路线的进度（那会静默跳过没巡检的点）。
SCHEMA_VERSION = 2


@dataclass
class Progress:
    """一次巡检的可恢复进度。"""

    route_name: str
    mode: int
    repeat_count: int
    loop: int = -1
    cursor: int = 0
    completed: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)
    # 当前这一轮的实际访问顺序（路线下标）。RANDOM 模式每轮重新洗牌，
    # 只有把顺序一起存下来，续跑时游标才对得上原来的那个点。
    order: List[int] = field(default_factory=list)
    # 点位组成（point_id 顺序）。同名路线被增删或重排后指纹会变，
    # 续跑自动作废——否则 job_id / cursor 都是「下标」语义，会张冠李戴。
    points: List[str] = field(default_factory=list)
    schema: int = SCHEMA_VERSION
    updated_at: str = ''

    @property
    def fingerprint(self) -> str:
        """路线指纹，用来判断快照还能不能用于当前路线。"""
        return fingerprint(self.route_name, self.mode, self.repeat_count,
                           self.points)

    def to_dict(self) -> dict:
        """转成可序列化的 dict。"""
        data = asdict(self)
        data['updated_at'] = datetime.now().isoformat(timespec='seconds')
        return data


def fingerprint(route_name: str, mode: int, repeat_count: int,
                points: Optional[Sequence[str]] = None) -> str:
    """路线指纹：路线名 + 模式 + 轮数 + 点位组成。"""
    points_key = '|'.join(str(item) for item in (points or []))
    raw = '%s|%d|%d|%s' % (route_name, int(mode), int(repeat_count), points_key)
    return hashlib.sha1(raw.encode('utf-8')).hexdigest()[:16]


def save(path: str, progress: Progress) -> bool:
    """原子写入进度快照。

    返回 ``True`` 表示写成功。写盘失败（目标被占用、只读、路径是目录…）
    只返回 ``False`` 并清掉临时文件，不抛异常：调用点在巡检主循环里，
    一次写盘失败不该让整轮巡检以 `internal error` 收场。
    """
    directory = os.path.dirname(os.path.abspath(os.path.expanduser(path)))
    temporary_name = ''
    try:
        os.makedirs(directory, exist_ok=True)
        payload = progress.to_dict()
        handle = tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=directory, prefix='.patrol_progress.',
            suffix='.tmp', delete=False)
        temporary_name = handle.name
        with handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, os.path.expanduser(path))
        return True
    except Exception:  # noqa: BLE001 - 进度写不进去不能拖垮巡检
        _remove_quietly(temporary_name)
        return False


def _remove_quietly(path: str) -> None:
    """删文件并吞掉所有异常（清理临时文件/进度文件用）。"""
    if not path:
        return
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    except OSError:
        pass


def load(path: str) -> Optional[Progress]:
    """读进度快照；文件不存在或内容不合法时返回 ``None``（当作没有进度）。"""
    real_path = os.path.expanduser(path)
    if not os.path.exists(real_path):
        return None
    try:
        with open(real_path, 'r', encoding='utf-8') as stream:
            data = json.load(stream)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    if data.get('schema') != SCHEMA_VERSION:
        return None
    try:
        progress = Progress(
            route_name=str(data['route_name']),
            mode=int(data['mode']),
            repeat_count=int(data['repeat_count']),
            loop=int(data.get('loop', -1)),
            cursor=int(data.get('cursor', 0)),
            completed=[str(item) for item in data.get('completed', [])],
            skipped=[str(item) for item in data.get('skipped', [])],
            order=[int(item) for item in data.get('order', [])],
            points=[str(item) for item in data.get('points', [])],
            updated_at=str(data.get('updated_at', '')),
        )
    except (KeyError, TypeError, ValueError):
        return None
    # 范围校验：负游标会让 plan 用负下标取到别的点，负轮次同理。
    if progress.loop < -1 or progress.cursor < 0 or progress.repeat_count < 0:
        return None
    return progress


def resumable(progress: Optional[Progress], route_name: str, mode: int,
              repeat_count: int, points: Optional[Sequence[str]] = None) -> bool:
    """判断已有快照能否用于本次请求（点位组成变了也不算可续跑）。"""
    if progress is None:
        return False
    return progress.fingerprint == fingerprint(route_name, mode, repeat_count,
                                              points)


def clear(path: str) -> None:
    """删除进度文件（正常跑完一轮后调用）。"""
    _remove_quietly(os.path.expanduser(path))
