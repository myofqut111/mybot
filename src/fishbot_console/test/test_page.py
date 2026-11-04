#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""page.py：页面必须自带全部资源（机器人常常没外网），且元素 id 和 JS 对得上。"""
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fishbot_console import page  # noqa: E402

REQUIRED_IDS = [
    'bState', 'bBatt', 'bPose', 'bCam', 'bErr', 'map', 'cam',
    'navX', 'navY', 'btnSendNav', 'btnCancelNav', 'btnCapture', 'camKind',
    'patrolRoute', 'patrolMode', 'patrolRepeat', 'btnPatrol',
    'btnPatrolCancel', 'mapName', 'btnMapStart', 'btnMapStop', 'btnMapSave',
    'logList', 'logLevel', 'runsBody', 'chatLog', 'askBox', 'btnAsk',
    'teleopPad', 'teleSpeed', 'teleLabel',
    'chkCostmap', 'chkScan', 'chkPlan', 'mapCoord', 'zoomLabel',
    'btnEstop', 'estopBar',
]


class TestPage(unittest.TestCase):
    def setUp(self):
        self.html = page.HTML

    def test_is_html(self):
        self.assertTrue(self.html.lstrip().startswith('<!doctype html>'))
        self.assertIn('</html>', self.html)

    def test_all_ids_exist(self):
        for element_id in REQUIRED_IDS:
            self.assertIn('id="%s"' % element_id, self.html,
                          'missing id %s' % element_id)

    def test_no_external_resources(self):
        # 不允许任何 http(s) 外链（CDN/字体/图标），否则没网时页面会卡住
        external = re.findall(r'(?:src|href)="https?://[^"]+"', self.html)
        self.assertEqual(external, [])

    def test_script_is_balanced(self):
        # 一个内联脚本 + 三个外链（leaflet / roslib / eventemitter2）。
        # 断言的是"开闭对得上"，不是"只有一个 script 标签"——多加一个 SDK
        # 不该让这条测试红，漏掉闭合标签才该红。
        self.assertEqual(self.html.count('<script>'), 1)
        self.assertEqual(self.html.count('<script src='), 3)
        self.assertEqual(self.html.count('</script>'), 4)
        self.assertEqual(self.html.count('<style>'), 1)
        self.assertEqual(self.html.count('</style>'), 1)

    def test_api_paths_match_server(self):
        for path in ('/api/state', '/api/logs', '/api/map.png',
                     '/api/frame.jpg', '/api/runs', '/api/nav',
                     '/api/patrol', '/api/capture', '/api/mapping',
                     '/api/ask', '/api/cancel_nav', '/api/patrol/cancel',
                     '/api/teleop', '/api/initialpose'):
            self.assertIn(path, self.html, 'page never calls %s' % path)

    def test_map_layers_and_scan_present(self):
        """RViz 对齐的三样东西必须真的在页面里，否则只是后端有接口。"""
        self.assertIn('id="chkCostmap"', self.html)
        self.assertIn('id="chkScan"', self.html)
        self.assertIn('/api/layers.png', self.html)
        self.assertIn('/api/scan', self.html)
        self.assertIn('LOADLAYERS', self.html.upper().replace('loadLayers', 'LOADLAYERS'))

    def test_rviz_like_interaction(self):
        """手感对齐 RViz 的硬指标（Leaflet 版）。

        换成 Leaflet 之后，滚轮锚定缩放、中键平移、双指缩放都由库负责，
        我们只需要保证：简单坐标系（1 单位 = 1 米）、拖动可开关、
        导航/位姿是"按下-拖出-松手"、代价层和点云各自能关。
        """
        self.assertIn('L.CRS.Simple', self.html)
        self.assertIn('LF.dragging.enable()', self.html)
        self.assertIn('LF.dragging.disable()', self.html)
        self.assertIn("LF.on('zoomend'", self.html)
        self.assertIn("LF.on('mousedown', onDown)", self.html)
        self.assertIn("LF.on('mouseup', onUp)", self.html)
        self.assertIn('chkCostmap', self.html)
        self.assertIn('chkScan', self.html)

    def test_map_drag_is_not_rubber_band(self):
        """拖完弹回原位，是 maxBoundsViscosity 默认值 0 造成的。

        用户在 RViz 里没有这种手感，会直接认为"拖不动"。这三条是修法的全部：
        硬边界（viscosity=1）、边界留余量（pad）、只在首次自动适应窗口。
        """
        self.assertIn('maxBoundsViscosity:1.0', self.html.replace(' ', ''))
        # 注意用 box.pad()：mapBounds() 是普通数组，数组没有 .pad()
        self.assertIn('setMaxBounds(box.pad(', self.html)
        self.assertIn('if(!MAPM.fitted)', self.html)

    def test_map_meta_goes_to_mapm(self):
        """地图元数据必须写到 MAPM：mapBounds() 读的是它。

        曾经写成 Object.assign(MAP, ...)，MAP 是个遗留对象，于是 MAPM 里
        w/h/res/ox/oy 一直是 0，叠加层边界退化成 [[0,0],[0,0]] —— 页面提示
        已经隐藏、地图却没有，用户看到的就是"没有地图"。
        """
        self.assertIn('Object.assign(MAPM', self.html)
        self.assertNotIn('Object.assign(MAP,', self.html)

    def test_browser_errors_are_reported(self):
        """前端异常要回传服务端，否则页面上的报错只能靠截图猜。"""
        self.assertIn('/api/client_log', self.html)
        self.assertIn("addEventListener('error'", self.html)
        self.assertIn("addEventListener('unhandledrejection'", self.html)

    def test_map_bounds_not_treated_as_latlngbounds(self):
        """mapBounds() 返回普通数组：不能直接 .pad()。

        浏览器日志实测：`mapBounds(...).pad is not a function at loadMap` —— 它把
        后面的 setMaxBounds/fitBounds 全打断了，于是"图加上了但视野没定位"，
        页面依旧一片空白。要用 L.latLngBounds 包一层。
        """
        self.assertNotIn('mapBounds().pad', self.html)
        self.assertIn('mapLLBounds()', self.html)
        self.assertIn('L.latLngBounds(mapBounds())', self.html)
        # 没有初始视野时调 fitBounds 会报 "Set map center and zoom first."
        self.assertIn('setView(', self.html)

    def test_no_undefined_helper_calls(self):
        """页面里被调用的助手函数必须真的定义过（note() 就曾漏定义）。"""
        for name in ('toast', 'api', 'setBadge', 'bubble', 'ensureMap',
                     'mapBounds', 'mapLLBounds', 'robotIcon', 'goalIcon'):
            self.assertIn('function %s(' % name, self.html,
                          'helper %s is never defined' % name)
        self.assertNotIn('note(', self.html)

    def test_map_click_sends_nav(self):
        # 换 Leaflet 之后不再手写 canvas 事件，改由 Leaflet 的 mousedown/mouseup
        # 回调驱动：按下定位置、拖出朝向、松手发目标。
        self.assertIn("LF.on('mousedown', onDown)", self.html)
        self.assertIn("LF.on('mouseup', onUp)", self.html)
        self.assertIn('sendNav(done.x, done.y, yaw)', self.html)

    def test_live_stream_wired(self):
        """rosbridge 实时流：组件要引进来，/tf 和 /scan 要真的订阅，还要能退回轮询。"""
        self.assertIn('/static/roslib.min.js', self.html)
        self.assertIn('/static/eventemitter2.min.js', self.html)
        self.assertIn('id="bLive"', self.html)
        self.assertIn('initLive', self.html)
        self.assertIn("name:'/tf'", self.html)
        self.assertIn("name:'/tf_static'", self.html)
        self.assertIn("name:'/scan'", self.html)
        self.assertIn('if(LIVE.ok) return', self.html)

    def test_teleop_has_deadman(self):
        # 手动控制必须有"松手即停"和页面失焦停，否则手机切走车会一直跑
        self.assertIn("addEventListener('pointerup'", self.html)
        self.assertIn("window.addEventListener('blur', teleStop)", self.html)
        self.assertIn("'/api/teleop'", self.html)

    def test_static_assets_exist(self):
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        static = os.path.join(os.path.dirname(here), 'fishbot_console', 'static')
        for name in ('leaflet.js', 'leaflet.css'):
            self.assertTrue(os.path.isfile(os.path.join(static, name)),
                            'missing static asset %s' % name)

    def test_estop_button_and_shortcut(self):
        """急停必须是"真按钮 + 快捷键 + 走服务端"三件事齐备。

        只在页面上画个红方块不算数：真正拦住车的是后端；请求失败要把按钮
        状态回退，否则会出现"页面显示已停、车还在动"这种最坏情况。
        """
        self.assertIn('id="btnEstop"', self.html)
        self.assertIn('id="estopBar"', self.html)
        self.assertIn("'/api/estop'", self.html)
        self.assertIn('estopToggle', self.html)
        self.assertIn("e.key === 'Escape'", self.html)
        # 请求失败要回退按钮状态
        self.assertIn('applyEstopUI(!wanted)', self.html)
        # 急停中禁用会动的按钮
        self.assertIn('btnSendNav', self.html)

    def test_estop_state_comes_from_server(self):
        # 状态以服务端为准，而不是按了按钮就一直显示"已停"
        self.assertIn('STATE.status', self.html)
        self.assertIn('.estop', self.html)
        self.assertIn('applyEstopUI(!!es.active)', self.html)


if __name__ == '__main__':
    unittest.main()
