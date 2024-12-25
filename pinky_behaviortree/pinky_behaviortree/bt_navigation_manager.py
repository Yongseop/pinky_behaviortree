import py_trees
from geometry_msgs.msg import PoseStamped
from .bt_navigate_to_goal import NavigateToGoal

class NavigationManager(NavigateToGoal):
    def __init__(self, name="NavigationManager", points=None):
        super().__init__(name=name)
        self.points = points if points else []
        self.current_point_index = 0
        
        # Blackboard 설정
        self.blackboard = self.attach_blackboard_client(name="NavigationManager")
        self.blackboard.register_key(key="current_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_future", access=py_trees.common.Access.WRITE) 
        self.blackboard.register_key(key="action_client", access=py_trees.common.Access.WRITE) 
        
    def update(self):
        """Navigation 업데이트"""
        if not self.points:
            self.node.get_logger().error("No points to navigate to")
            return py_trees.common.Status.FAILURE
            
        # 현재 목표점 설정
        self.goal_pose = self.points[self.current_point_index]
        self.blackboard.current_goal = self.current_point_index
        
        # Navigation 실행
        status = super().update()
        
        # Navigation 성공 시 다음 포인트로
        if status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info(f"Reached point {self.current_point_index}")
            self.current_point_index = (self.current_point_index + 1) % len(self.points)
            self.reset_navigation_state()
            return py_trees.common.Status.RUNNING
            
        return status