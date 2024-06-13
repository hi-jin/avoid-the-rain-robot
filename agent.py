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

import numpy as np
import stable_baselines3 as sb3
import gym
import gym.spaces
import rain_game as game
import wandb
from wandb.integration.sb3 import WandbCallback


class AvoidTheRainEnv(gym.Env):
    def __init__(
        self,
        show_simulation_at_every_episode=10,
    ):
        super(AvoidTheRainEnv, self).__init__()

        # --------- define spaces for rl algorithm
        self.action_space = gym.spaces.MultiDiscrete([3, 3])
        self.observation_space = gym.spaces.Box(
            low=0,
            high=255,
            shape=(480, 640, 4),
        )

        # --------- define camera related settings
        self.simulation_step = 0
        self.update_on_every = 3

        # --------- my variables
        self.global_episode = 0
        self.show_simulation_at_every_episode = show_simulation_at_every_episode
        self.episode_start_pos = None
        self.episode_robot_imgs = []

    def step(self, action: np.ndarray):
        self.simulation_step += 1
        while self.simulation_step % self.update_on_every != 0:
            game.p.stepSimulation()
            self.simulation_step += 1

        # ---------- step the game
        obs = game.update_camera(game.robot_id)
        left = action[0] - 1
        right = action[1] - 1

        left *= 50
        right *= 50

        game.move_robot_wheel(game.robot_id, left, right)
        match game.game_step():
            case game.GameState.ALIVE:
                done = False
            case game.GameState.DEAD:
                done = True

        # ---------- calculate reward (more distance == more reward)
        robot_pos = game.get_robot_pos(game.robot_id)
        distance = np.linalg.norm(np.array(robot_pos) - np.array(self.episode_start_pos))
        reward = abs(distance) * 0.1
        # but if dead, then 0
        if done:
            reward = 0
        
        robot_img = game.show_robot_current_image(game.robot_id)
        robot_img = np.array(robot_img) # (480, 640, 4)
        robot_img = robot_img.transpose(2, 0, 1) # (4, 480, 640)
        self.episode_robot_imgs.append(robot_img)
        wandb.log({
            "reward": reward,
            "distance": distance,
        })

        return obs, reward, done, {}

    def reset(self):
        # ---------- save the episode images to video
        if self.episode_robot_imgs:
            wandb.log({
                "episode": self.global_episode,
                "robot_imgs": wandb.Video(np.array(self.episode_robot_imgs), fps=20),
            })
            self.episode_robot_imgs = []

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
        self.simulation_step = 0
        game.reset_all(self.render_mode)
        self.episode_start_pos = game.get_robot_pos(game.robot_id)
        if not game.robot_id:
            return None
        else:
            return game.update_camera(game.robot_id)

    def render(self):
        pass


def train():
    wandb.init(
        project="avoid-the-rain",
        name="ppo",
        monitor_gym=True,
        sync_tensorboard=True,
        save_code=True,
    )

    env = AvoidTheRainEnv(show_simulation_at_every_episode=-1)
    model = sb3.PPO("MlpPolicy", env, verbose=1)
    model.learn(
        total_timesteps=1000000,
        callback=WandbCallback(
            model_save_path="./models/",
            model_save_freq=10000,
            verbose=2,
        ),
    )
    model.save("avoid_the_rain")


def test():
    env = AvoidTheRainEnv(show_simulation_at_every_episode=1)
    model = sb3.PPO.load("./models/model.zip")
    obs = env.reset()
    done = False
    while not done:
        action, _states = model.predict(obs)
        print(action)
        obs, reward, done, info = env.step(action)


if __name__ == "__main__":
    train()
    # test()
