"""Create the Isaac Sim 5.1 OmniGraph publishers for all Wheel Bot cameras."""

from __future__ import annotations

from franzi_sim.cameras import CAMERAS, validate_camera_specs


def add_ros2_camera_publishers(
    robot_prim_path: str = "/World/WheelBot",
    graph_path: str = "/World/ROS2CameraPublishers",
) -> None:
    """Publish RGB images and CameraInfo for all four declared camera sensors.

    The ROS 2 bridge extension must be enabled before this function is called.
    Imports stay local so non-Isaac unit tests can still import the project.
    """

    import omni.graph.core as og
    from pxr import Sdf

    validate_camera_specs()
    nodes: list[tuple[str, str]] = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
    values: list[tuple[str, object]] = []
    connections: list[tuple[str, str]] = []

    for camera in CAMERAS:
        suffix = camera.name
        render_node = f"Render_{suffix}"
        image_node = f"Image_{suffix}"
        info_node = f"Info_{suffix}"
        camera_path = (
            f"{robot_prim_path}/{camera.parent_link}/"
            f"{camera.camera_mount_prim}/camera"
        )

        nodes.extend(
            [
                (render_node, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                (image_node, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (info_node, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
        )
        values.extend(
            [
                (f"{render_node}.inputs:cameraPrim", [Sdf.Path(camera_path)]),
                (f"{render_node}.inputs:width", camera.width_px),
                (f"{render_node}.inputs:height", camera.height_px),
                (f"{image_node}.inputs:type", "rgb"),
                (f"{image_node}.inputs:topicName", camera.topic),
                (f"{image_node}.inputs:frameId", camera.frame_id),
                (f"{image_node}.inputs:queueSize", 5),
                (f"{info_node}.inputs:topicName", camera.camera_info_topic),
                (f"{info_node}.inputs:frameId", camera.frame_id),
                (f"{info_node}.inputs:queueSize", 5),
            ]
        )
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", f"{render_node}.inputs:execIn"),
                (f"{render_node}.outputs:execOut", f"{image_node}.inputs:execIn"),
                (f"{render_node}.outputs:execOut", f"{info_node}.inputs:execIn"),
                (f"{render_node}.outputs:renderProductPath", f"{image_node}.inputs:renderProductPath"),
                (f"{render_node}.outputs:renderProductPath", f"{info_node}.inputs:renderProductPath"),
            ]
        )

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.SET_VALUES: values,
            og.Controller.Keys.CONNECT: connections,
        },
    )
