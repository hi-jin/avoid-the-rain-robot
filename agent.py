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

import gymnasium as gym
import numpy as np
import stable_baselines3 as sb3
import rain_game as game
import wandb
from wandb.integration.sb3 import WandbCallback
from stable_baselines3.common.vec_env import VecVideoRecorder, SubprocVecEnv
from stable_baselines3.common.monitor import Monitor


ACTION_SCALE = 50


class AvoidTheRainEnv(gym.Env):
    def __init__(
        self,
        show_simulation_at_every_episode=10,
    ):
        super(AvoidTheRainEnv, self).__init__()

        # --------- define spaces for rl algorithm
        self.action_space = gym.spaces.Box(
            low=-1,
            high=1,
            shape=(2,),
        )
        self.observation_space = gym.spaces.Dict(
            {
                "depth": gym.spaces.Box(
                    low=0,
                    high=255,
                    shape=(480, 640),
                    dtype=np.uint8,
                ),
                "prev_action": gym.spaces.Box(
                    low=-1,
                    high=1,
                    shape=(2,),
                )
            }
        )

        # --------- my variables
        self.global_episode = 0
        self.show_simulation_at_every_episode = show_simulation_at_every_episode
        self.episode_rewards = []

    def step(self, action: np.ndarray):
        # --------- do action
        action *= ACTION_SCALE
        left = action[0]
        right = action[1]

        game.move_robot_wheel(game.robot_id, left, right)
        game.p.stepSimulation()
        match game.game_step():
            case game.GameState.ALIVE:
                done = False
            case game.GameState.DEAD:
                done = True

        # ---------- get obs
        obs = game.update_camera(game.robot_id)
        obs = np.array(obs)  # (480, 640, 4)
        obs = {
            "depth": obs,
            "prev_action": action / ACTION_SCALE,
        }

        # ---------- calculate reward
        reward = self.calculate_reward(left, right)
        if done:
            reward = 0

        # ---------- save the episode rewards
        self.episode_rewards.append(reward)

        return obs, reward, done, False, {}

    def reset(self, seed=None):
        # ---------- determine whether to show simulation or not
        user_dont_want_to_show_simulation = self.show_simulation_at_every_episode == -1
        should_show = self.global_episode % self.show_simulation_at_every_episode == 0
        if not user_dont_want_to_show_simulation and should_show:
            print(f"Episode: {self.global_episode}")
            self.render_mode = "human"
        else:
            self.render_mode = "rgb_array"

        # ---------- reset the game
        self.global_episode += 1
        game.reset_all(self.render_mode)
        if not game.robot_id:
            return None, {}
        else:
            obs = game.update_camera(game.robot_id)
            obs = {
                "depth": obs,
                "prev_action": np.array([0, 0], dtype=np.float32),
            }
            return obs, {}

    def render(self, mode="rgb_array"):
        robot_img = game.show_robot_current_image(game.robot_id)
        robot_img = np.array(robot_img)  # (480, 640, 4)
        return robot_img

    def calculate_reward(self, left, right):
        return 1


def make_env():
    def _init():
        env = AvoidTheRainEnv(show_simulation_at_every_episode=-1)
        return Monitor(env)

    return _init


def train():
    run = wandb.init(
        project="avoid-the-rain",
        name="ppo",
        monitor_gym=True,
        sync_tensorboard=True,
        save_code=True,
    )

    num_envs = 4
    vec_env = SubprocVecEnv([make_env() for _ in range(num_envs)])
    vec_env.render_mode = "rgb_array"
    recorder = VecVideoRecorder(
        vec_env,
        "videos",
        record_video_trigger=lambda x: x % (50000 // num_envs) == 0,
        video_length=1000,
    )
    model = sb3.PPO(
        "MultiInputPolicy",
        recorder,
        verbose=1,
        tensorboard_log=f"./runs/{run.id}",
        ent_coef=0.01,
        n_steps=2048 // num_envs,
    )
    model.learn(
        total_timesteps=1000000,
        callback=WandbCallback(
            model_save_path="./models/",
            model_save_freq=10000,
            verbose=2,
        ),
    )
    run.finish()


def test():
    env = AvoidTheRainEnv(show_simulation_at_every_episode=1)
    model = sb3.PPO.load("./models/model.zip")
    obs, _ = env.reset()
    done = False
    while not done:
        action, _ = model.predict(obs)
        action *= ACTION_SCALE
        # action = (50, 50)
        obs, reward, done, truncation, info = env.step(action)
        # print(obs)


if __name__ == "__main__":
    try:
        train()
        # test()
    except Exception as e:
        print(e)
