"""
 Copyright (c) 2024 Hyungjin Ahn

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 """

import pybullet as p
import pybullet_data
from typing import List
import time
import numpy as np
from enum import Enum, auto


TOTAL_RAIN_COUNT = 100

is_already_initialized = False
spawned_rains = []
robot_id = None
plane_id = None


class GameState(Enum):
    ALIVE = 0
    DEAD = auto()


def reset_all(render_mode: str = "human"):
    global is_already_initialized, spawned_rains, robot_id, plane_id

    # ------------------------- initialize
    if p.isConnected():
        p.disconnect()
    p.connect(p.GUI) if render_mode == "human" else p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # ------------------------- reset
    spawned_rains = []
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    plane_id = p.loadURDF("plane.urdf")

    # ------------------------- spawn robot
    robot_id = create_robot([0, 0, 1])

    # ------------------------- spawn rain
    for _ in range(TOTAL_RAIN_COUNT):
        create_rain(spawned_rains, height=np.random.randint(5, 10))


def game_step() -> GameState:
    global spawned_rains, robot_id, plane_id

    # ------------------ check collision
    for rain_id in spawned_rains:
        # ---- check if rain meets robot
        contacts = p.getContactPoints(rain_id, robot_id)
        if contacts:
            return GameState.DEAD

        # ---- check if rain meets plane
        contacts = p.getContactPoints(rain_id, plane_id)
        if contacts:
            reset_rain_pos_near_the_robot(rain_id)

    # ------------------------- random rain (now will be created on initialize for better performance)
    # if len(spawned_rains) < TOTAL_RAIN_COUNT:
    #     create_rain(spawned_rains)

    return GameState.ALIVE


def create_robot(pos: List[float]) -> str:
    # ------------------------- create robot
    robot_id = p.loadURDF("r2d2.urdf", pos)

    return robot_id


def get_robot_pos(robot_id: str) -> List[float]:
    pos, _ = p.getBasePositionAndOrientation(robot_id)
    return pos


def move_robot_wheel(robot_id: str, left_wheel: float, right_wheel: float):
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=2,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=left_wheel,
    )
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=3,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=left_wheel,
    )

    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=6,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=right_wheel,
    )
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=7,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=right_wheel,
    )


def create_rain(spawned_rains, height=10):
    robot_pos_x = get_robot_pos(robot_id)[0]
    robot_pos_y = get_robot_pos(robot_id)[1]
    pos_x = np.random.uniform(-10, 10) + robot_pos_x
    pos_y = np.random.uniform(-10, 10) + robot_pos_y
    rain_id = p.loadURDF("sphere2.urdf", [pos_x, pos_y, height])
    spawned_rains.append(rain_id)


def reset_rain_pos_near_the_robot(rain_id):
    # move rain to the sky (near the robot)
    robot_pos_x = get_robot_pos(robot_id)[0]
    robot_pos_y = get_robot_pos(robot_id)[1]
    pos_x = np.random.uniform(-10, 10) + robot_pos_x
    pos_y = np.random.uniform(-10, 10) + robot_pos_y
    p.resetBasePositionAndOrientation(rain_id, [pos_x, pos_y, 10], [0, 0, 0, 1])


def update_camera(robot_id):
    # 로봇의 위치와 방향 가져오기
    robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
    # 로봇의 머리 위에 카메라 위치 설정 (로봇 위치 + 오프셋)
    camera_eye_position = [robot_pos[0], robot_pos[1], robot_pos[2] + 0.5]
    # 로봇의 바로 위를 바라보도록 카메라 방향 설정
    camera_target_position = [robot_pos[0], robot_pos[1], robot_pos[2] + 1.5]
    camera_up_vector = [0, 1, 0]

    # 카메라의 뷰 매트릭스와 프로젝션 매트릭스 계산
    view_matrix = p.computeViewMatrix(camera_eye_position, camera_target_position, camera_up_vector)
    fov = 60
    aspect = 640 / 480
    near_plane = 0.1
    far_plane = 100
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near_plane, far_plane)

    # 카메라 이미지 캡처
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        640, 480, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    return rgb_img


def show_robot_current_image(robot_id):
    robot_pos = get_robot_pos(robot_id)
    camera_eye_position = [robot_pos[0] + 3, robot_pos[1] + 3, robot_pos[2] + 3]
    camera_target_position = [robot_pos[0], robot_pos[1], robot_pos[2]]

    view_matrix = p.computeViewMatrix(camera_eye_position, camera_target_position, [0, 0, 1])
    fov = 60
    aspect = 640 / 480
    near_plane = 0.1
    far_plane = 100
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near_plane, far_plane)

    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        640, 480, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    return rgb_img


if __name__ == "__main__":
    reset_all("rgb_array")

    rendering_step = 0
    while True:
        p.stepSimulation()
        match game_step():
            case GameState.ALIVE:
                pass
            case GameState.DEAD:
                print("Game Over")
                reset_all()

        # 카메라 업데이트
        if rendering_step % 3 == 0:
            # rgb_img = update_camera(robot_id)
            robot_current = show_robot_current_image(robot_id)

        # move test
        # move_robot_wheel(robot_id, 50, 50)

        rendering_step += 1
        time.sleep(1 / 240)
