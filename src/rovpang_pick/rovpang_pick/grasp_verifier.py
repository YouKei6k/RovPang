
class GraspVerifier:
    def __init__(
        self,
        closed_empty_threshold=550,
        open_threshold=250
    ):
        self.closed_empty_threshold = closed_empty_threshold
        self.open_threshold = open_threshold

    def is_grasped(self, gripper_position):
        if gripper_position is None:
            return False

        return False
