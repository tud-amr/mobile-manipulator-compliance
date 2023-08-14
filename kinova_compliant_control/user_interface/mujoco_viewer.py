import importlib.resources as pkg_resources
import mujoco
import kinova.models as models


class MujocoViewer:
    """Provides the mujoco visualization of the robot."""

    def __init__(self) -> None:
        xml = str(pkg_resources.files(models) / "GEN3-LITE.xml")
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
