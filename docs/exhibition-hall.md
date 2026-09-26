# 展馆网格模型与人物模型（2026-09-25）

把展馆从「程序化生成的 16×12 m 长方体房间」换成 **Fuel 上的真实博物馆网格**，并加入参观者。
原来的简单展馆保留在 `config/exhibition_simple.json`，SLAM 仍然用它。

## 变更前后

| | 之前 | 现在 |
|---|---|---|
| 场地 | 代码生成的 4 面墙 + 地面，16×12 m | 导入 `iscas_museum` 网格，22.5×20.5 m，层高 4.0 m |
| 展品 | 4 张海报贴在生成的墙上 | 4 幅名画贴在真实墙面（北墙 2、西墙 1、南墙 1），支持任意朝向；2026-09-26 起为美术馆主题 |
| 人 | 无 | 3 个静态参观者 + 2 个行走 actor |
| 障碍 | 2 长椅 + 2 花盆（合成方块） | 馆内自带隔断展柜；合成方块已移除，`obstacles` 为空 |
| 导航静态地图 | 由 `room` 尺寸直接算 | 由网格烘焙出的 320 个矩形（`config/hall_nav_obstacles.json`） |
| 配置 schema | 1 | 2（1 仍然支持，见下） |

## 导入的模型

全部已下载到 `~/.gz/fuel`，**离线可用**，不需要联网。

| 用途 | Fuel 模型 | 说明 |
|---|---|---|
| 展馆 | `OpenRobotics/iscas_museum` | CC-BY 4.0，8.2 MB DAE，38k 三角面 |
| 静态参观者 | `OpenRobotics/Casual female` | 静态，盒式碰撞 0.76×0.33×1.77，最省资源 |
| 静态参观者 | `OpenRobotics/FemaleVisitor` | 静态，带独立低模碰撞网格 |
| 静态参观者 | `OpenRobotics/Standing person` | 原模型非静态，本项目在 `<include>` 里强制 `static=true` |
| 行走 actor | `OpenRobotics/MaleVisitorPhone` | 骨骼动画皮肤 |
| 行走 actor | `Luca/FemaleVisitorWalk` | 骨骼动画皮肤 |

换机器时重新拉取：

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/iscas_museum"
```

## 这个网格只有墙

实测（`tools/hall/probe_museum.py`）：`iscas_museum` **没有地面，也没有屋顶**，z<0.15 的面只是墙脚和柱脚。所以

- **地面仍由 `exhibition_world.py` 生成**（22×24 m 板，顶面 z=0）。雷达需要一致的地面回波，不能少。
- 俯视相机可以放在馆外上方直接看进馆内（`overview_camera` 位于 z=16）。

## 导航静态地图怎么来的

`nav_grid.OccupancyGrid` 按矩形规划，不吃网格。所以离线把网格烘焙成矩形：

1. `probe_museum.py` — 解析 DAE，按三角面采样（不是只取顶点，否则大墙面只剩 4 个点），套用节点矩阵和 `model.sdf` 的 yaw −1.57，输出世界坐标点云。
2. `bake_nav_obstacles.py` — 建 0.1 m 占据栅格，按 0.55 m 半径膨胀后从原点洪水填充，只烘焙机器人可达区域外壳（更远的几何本来就到不了），贪心分解成矩形。
3. `verify_bake.py` — 把矩形重新栅格化，与网格直接比可达集合。

**验证结果：可达区域完全一致。**

```
mesh reachable      :   201.32 m^2
baked reachable     :   201.32 m^2
LEAK (through walls):     0.00 m^2   <- 必须为 0
lost (over-blocked) :     0.00 m^2
```

结果写进 `config/hall_nav_obstacles.json`（320 个矩形，36 KB），由 `scene_config.load_scene()` 通过 `nav_obstacles_file` 自动并入配置。

> 第一次烘焙用了 `--min-area 0.09`，剩下 657 个碎格没覆盖，`verify_bake.py` 报出 **3.75 m² 泄漏**——机器人能穿墙。改成 `--min-area 0.01` 全覆盖才归零。这一步不能省。

## 布局

坐标全部经 `tools/hall/check_layout.py` 对网格实测，不是估的。

| 展位 | 展品 | 位置 | 站位 | 站位离障碍 |
|---|---|---|---|---|
| 1 | E01 清明上河图（挂画，北墙） | (−3.70, 10.95) | (−3.70, 9.10) | 2.00 m |
| 2 | E02 千里江山图（挂画，北墙） | (3.90, 10.95) | (3.90, 9.10) | 1.84 m |
| 3 | E03 星月夜（挂画，西墙） | (−8.95, 9.10) | (−7.10, 9.10) | 2.00 m |
| 4 | E04 神奈川冲浪里（挂画，南墙） | (0.70, −10.45) | (0.70, −8.60) | 1.91 m |
| 5 | E05 芥子园画传（立体，古籍） | (3.40, 7.00) | (3.40, 5.00) | 1.61 m |
| 6 | E06 青花瓷瓶（立体） | (−5.10, −7.30) | (−3.20, −7.30) | 1.50 m |

出生点 (0.00, −9.00)，朝北，靠近南入口。

人物：静态参观者在 (5.50, 4.50)、(−6.00, 5.00)、(0.90, 5.60)；两个 actor 分别在北厅和南廊循环巡游。

`tools/hall/check_tour.py` 用**真实的 `OccupancyGrid`** 规划，6 个展位全部可达，最远 32.3 m / 0.72 s。

## 配置 schema v2

`schema_version: 1` 仍然有效（`exhibition_simple.json` 就是），新增的都是可选项：

| 键 | 作用 |
|---|---|
| `hall` | `{name, model_uri, pose}`。**存在时不生成四面墙**，改为 `<include>` 网格；地面照常生成。缺省则退回原来的长方体房间。 |
| `nav_obstacles_file` | 指向同目录的烘焙文件；`load_scene()` 负责并入。 |
| `nav_obstacles` | 只给规划器用的矩形，**不渲染**（网格已经画出来了）。参与站位/出生点的余量检查，但不参与模型重叠检查——海报本来就贴着墙。 |
| `exhibits[].yaw` | 展品朝向。海报正面朝本地 −y，yaw=0 朝北墙、π/2 朝西墙、π 朝南墙。 |
| `people[]` | `kind: "static"` → 带碰撞的 `<include>`；`kind: "actor"` → 带 `waypoints` 和 `speed` 的骨骼动画，自动闭合成循环轨迹。 |

`scene_config.world_footprint()` 是几何单一真相：世界生成器和 `nav_grid` 都用它算旋转后的轴对齐占地，避免「画在一处、绕开另一处」。

新增校验：站位 yaw 必须朝向展品（误差 >12° 报错）。

## 两个限制

**行走的 actor 雷达看不见。** Gazebo 的 `<actor>` 没有碰撞体，只有外观和动画。摄像头拍得到（对任务三的视觉识别有用），但 `/nav_scan` 不返回他们，避障不会因他们触发。真正参与避障的是 3 个静态参观者——带碰撞的刚体，已进导航栅格。要做「躲开走动的人」，得换成带碰撞的模型 + `TrajectoryFollower` 插件驱动，物理驱动的人形容易摔倒，尚未实现。

**合成长椅花盆已移除。** 馆内自带隔断和展柜。需要长椅可以按实测空地补回 `obstacles`。

## 踩过的坑：Fuel 路径里的空格会让世界永远起不来

现象：Gazebo 打开是黑屏，Entity Tree 空的，服务器 100% CPU 空转，报

```
[error] [SceneBroadcaster.cc:781] Timed out waiting for state
```

不是渲染问题——物理线程根本没启动。用 `tools/hall/bisect_world.py` 逐项拆：

| 变体 | 结果 |
|---|---|
| 完整世界 | 挂死 >90 s |
| 去掉两个 actor | 12.0 s ✅ |
| 去掉俯视相机 | 4.0 s ✅ |
| 只留 `walker_gallery` | 12.6 s ✅ |
| 只留 `walker_corridor` | 挂死 >90 s ❌ |
| 完整世界改用 `ogre` 引擎 | 仍然挂死 ❌ |

元凶是单个模型：`walker_corridor` 原本用 `OpenRobotics/male visitor`，Fuel 路径**带空格**，编码成 `male%20visitor`。Gazebo 的网格解析器碰到这个 URL 不报错，直接卡住。换成 `Luca/FemaleVisitorWalk` 后完整世界 12.4 s 启动。

注意 `<include>` 的 URI 带空格**没问题**（`Casual female`、`Standing person` 都正常）。只有 actor 的 `<skin><filename>` 会挂。所以 `validate_layout` 只对 `skin_uri` 拒绝空格和 `%20`，并附带 2 个回归测试。

## 工具

都在 `tools/hall/`，可独立运行。首次使用先跑 `probe_museum.py` 生成点云缓存（200 MB，已 gitignore）。

| 脚本 | 用途 |
|---|---|
| `probe_museum.py` | 解析网格，打印俯视占据图，缓存世界坐标点云 |
| `plan_layout.py` | 膨胀 + 洪水填充，打印可达区域，列出最开阔的位置 |
| `propose_spots.py` | 搜索「平直墙面 + 前方空地 + 可站立站位」三者同时满足的展品锚点 |
| `bake_nav_obstacles.py` | 烘焙导航矩形（改配置后须带 `--min-area 0.01`） |
| `verify_bake.py` | 逐格对比矩形与网格的可达集合，**泄漏必须为 0** |
| `check_layout.py` | 用 `--point x y` 查单点余量，或 `--config` 全量校验布局 |
| `check_tour.py` | 用真实 `OccupancyGrid` 规划全程，证明 6 个展位可达 |
| `bisect_load.py` / `bisect_world.py` | 定位是哪个模型让 Gazebo 起不来 |

换展馆模型后的完整流程：

```bash
cd /mnt/d/g1_guide
python3 tools/hall/probe_museum.py                              # 重建点云缓存
python3 tools/hall/bake_nav_obstacles.py --min-area 0.01 > tools/hall/nav_obstacles.json
python3 tools/hall/verify_bake.py                               # 泄漏必须为 0
python3 tools/hall/propose_spots.py                             # 挑新的展品墙面
# 改 config/exhibition.json 后：
python3 tools/hall/check_layout.py --config ws/src/g1_guide/config/exhibition.json
python3 tools/hall/check_tour.py
```

## 验证状态

| 项 | 状态 |
|---|---|
| 烘焙矩形与网格可达区域一致 | ✅ 泄漏 0，过度封堵 0 |
| 6 个展位规划可达（离线） | ✅ 最远 32.3 m / 0.72 s |
| 完整世界在 Gazebo 里起步进仿真 | ✅ 12.4 s |
| 单元测试 | ✅ 166 项 |
| `exhibition_simple.json` 向后兼容 | ✅ 仍生成长方体房间，6/6 可达 |
| Gazebo 里肉眼确认画面 | ✅ 2026-09-25 用户确认 |
| **机器人按 1–6 在新馆里到站** | ⬜ 待用户验证 |
