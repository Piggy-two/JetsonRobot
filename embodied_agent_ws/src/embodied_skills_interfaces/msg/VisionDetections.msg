# 某一帧的检出结果。**每收到一帧发一条**（与 LiDAR 原语的 `/embodied/lidar/front` 同节奏）。

builtin_interfaces/Time stamp
# ⚠️ 这是**本节点收到这一帧的时间**，**不是**图像 `header.stamp`。
#    理由（#23 / #24）：厂商 `usb_cam` 的图像戳**比真实采集时刻早**，而且陈旧量
#    **不是常数**（随每次开机的时钟跳变，实测过一次 338.6 秒）。
#    直接透传它，会让"这个检出是什么时候看到的"这个问题的答案错几分钟。

float32 image_quality
# 中心取样区的清晰度（Laplacian 方差，与 `tools/focus_assist.py` 同一口径）。
# ⚠️ 它是**相对**指标：画面里本来就没纹理（白墙、天花板）时天然很低 ——
#    那是"没东西可聚焦"，不是相机的错。所以它是**参考量**，
#    "能不能据此回答"的规则在 `~/find_in_view` 的语义里（下一步看那个）。

VisionDetection[] objects
