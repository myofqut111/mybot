"""期望值的"基线"：这一趟看到了什么，就当作下一趟的期望值。

以前 ``expect``（present/absent）只能在录点时手选，一趟趟点下来很烦，
而且大多数人根本不选——于是报告里的召回率/精确率永远是 ``null``。

现在的做法是"边跑边学"：

* 第 1 趟：点位没有手写 expect，就按这趟实际看到的结果记成基线；
* 第 2 趟起：没有手写 expect 的点位自动沿用基线，报告里直接出
  召回率/误报率，不用再点任何东西。

两条保护：

1. **手写的永远优先**。录点时写了 present/absent，基线不参与，也不会被覆盖。
2. 默认策略是 ``fill``——只补空缺，不覆盖已学过的点。否则某一趟误检一次
   （该有的没看到），基线就被改写成 absent，之后每次都"正确"地漏检，
   召回率反而永远好看。想跟着场景变化更新，把 ``baseline_policy`` 设成
   ``update``。
"""

import json
import os
import tempfile
from datetime import datetime

EXPECT_PRESENT = 'present'
EXPECT_ABSENT = 'absent'

# 只补空缺 / 每趟覆盖
POLICY_FILL = 'fill'
POLICY_UPDATE = 'update'
POLICIES = (POLICY_FILL, POLICY_UPDATE)

DEFAULT_PATH = '~/.ros/fishbot_patrol_expect.json'
VERSION = 1


def store_path(path: str = '') -> str:
    """基线文件的实际路径（支持 ``~``）。"""
    return os.path.expanduser(path or DEFAULT_PATH)


def empty() -> dict:
    """一份空基线。"""
    return {'version': VERSION, 'points': {}}


def load(path: str = '') -> dict:
    """读基线。文件不存在/坏了都退回空基线——巡检不能因为一个缓存文件起不来。"""
    target = migrate_legacy_path(store_path(path))
    try:
        with open(target, encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return empty()
    if not isinstance(data, dict) or not isinstance(data.get('points'), dict):
        return empty()
    data.setdefault('version', VERSION)
    return data


def save(data: dict, path: str = '') -> str:
    """原子写回（先写 .tmp 再 replace），中途断电不会留下半个 JSON。"""
    target = store_path(path)
    directory = os.path.dirname(target)
    if directory:
        os.makedirs(directory, exist_ok=True)
    handle, temp = tempfile.mkstemp(prefix='.expect-', dir=directory or '.')
    try:
        with os.fdopen(handle, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
        os.replace(temp, target)
    except Exception:
        try:
            os.unlink(temp)
        except OSError:
            pass
        raise
    return target


def key(route_name: str, point_id: str) -> str:
    """基线的键：同一条路线上同名点位才算同一个点。"""
    return '%s/%s' % (str(route_name), str(point_id))


def lookup(data: dict, route_name: str, point_id: str) -> str:
    """查这个点的基线期望值，没有就是空串。"""
    points = (data or {}).get('points') or {}
    entry = points.get(key(route_name, point_id))
    if not isinstance(entry, dict):
        return ''
    value = str(entry.get('expect') or '')
    return value if value in (EXPECT_PRESENT, EXPECT_ABSENT) else ''


def resolve(manual: str, stored: str):
    """决定这个点用哪个期望值，并说明来源。

    返回 ``(expect, source)``，``source`` 是 ``manual`` / ``baseline`` / ``''``。
    """
    manual = str(manual or '')
    if manual in (EXPECT_PRESENT, EXPECT_ABSENT):
        return manual, 'manual'
    if stored in (EXPECT_PRESENT, EXPECT_ABSENT):
        return stored, 'baseline'
    return '', ''


def learn(data: dict, route_name: str, point_id: str, detected: bool,
          policy: str = POLICY_FILL, source_run: str = '',
          now: str = '') -> bool:
    """把"这趟看到了没有"记成基线，返回是否真的改了。

    ``policy='fill'`` 时已存在的点不动；``update`` 时按最新一次覆盖。
    """
    if policy not in POLICIES:
        policy = POLICY_FILL
    points = data.setdefault('points', {})
    name = key(route_name, point_id)
    current = points.get(name) if isinstance(points.get(name), dict) else {}
    if current and policy == POLICY_FILL:
        return False
    expect = EXPECT_PRESENT if detected else EXPECT_ABSENT
    samples = int(current.get('samples') or 0) + 1
    entry = {
        'expect': expect,
        'learned_at': now or datetime.now().isoformat(timespec='seconds'),
        'source_run': str(source_run),
        'samples': samples,
    }
    points[name] = entry
    return current.get('expect') != expect or not current


def describe(data: dict) -> str:
    """给人看的一句话：学了多少个点、其中几个 present。"""
    points = (data or {}).get('points') or {}
    present = sum(1 for item in points.values()
                  if isinstance(item, dict) and item.get('expect') == EXPECT_PRESENT)
    return '%d 个点（%d 个应该有、%d 个应该没有）' % (
        len(points), present, len(points) - present)


def migrate_legacy_path(path):
    """改名兼容：老机器上还叫 fishbot_*.json，首次运行时原地复制一份新名字。"""
    import os as _os
    path = _os.path.expanduser(str(path))
    name = _os.path.basename(path)
    legacy = name.replace('fishbot_', 'fishbot_')
    if legacy == name:
        return path
    legacy_path = _os.path.join(_os.path.dirname(path), legacy)
    if not _os.path.exists(path) and _os.path.exists(legacy_path):
        try:
            import shutil
            shutil.copy2(legacy_path, path)
        except OSError:
            return legacy_path
    return path
