import gymnasium
import CustomEnv

def test_env(): #used to test the enviroment with random actions
    env = gymnasium.make("airsim-drone-dynamic-v0")
    
    for episode in range(10):
        done = False
        score = 0
        print(env.reset())
        img, obs = env.reset()

        while not done:
            action = env.action_space.sample() 
            # with gymnasium, step returns 5 value instead of 4
            # obs, reward, done, truncated, info = env.step(action)
            obs, reward, terminated, truncated, info = env.step(action)
            
            #print("Enter manual input")
            #input_a = input()
            #obs, reward, done, truncated, info = env.step(int(input_a))

            score += reward
        
        if episode % 10 == 0:
            print("Episode: {} Score: {}".format(str(episode), str(score)))
            
    env.reset()
    env.disconnect()
    return  

test_env()


