from pynput import keyboard

class KeyboardHandle:
    def __init__(self):
        self.running = False

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=True)
        
        self.run_loco_signal = False
        self.run_squat_signal = False
        self.stopgait_signal = False
        self.left_hand_grasp_state = True
        self.right_hand_grasp_state = True
        self.start_signal = False
        self.run_signal = False
        self.damping_signal = False
        self.move_x = 0.0
        self.move_y = 0.0
        self.turn_x = 0.0
        self.turn_y = 0.0

        self.move_speed = 2.0
        self.turn_speed = 4.0

    def start_listener(self):
        print("Started keyboard listener")
        self.running = True
        self.listener.start()

    def stop_listener(self):
        print("Stopped keyboard listener")
        self.running = False
        self.listener.stop()
            
    def on_press(self, key):
        try:
            # Signals
            if key == keyboard.KeyCode(char="1"):
                self.run_loco_signal = True
                print("run_loco_signal")
            elif key == keyboard.KeyCode(char="2"):
                self.run_squat_signal = True
                print("run_squat_signal")
            elif key == keyboard.KeyCode(char="3"):
                self.stopgait_signal = True
                print("stopgait_signal")
            elif key == keyboard.KeyCode(char="4"):
                self.left_hand_grasp_state = True
                print("left_hand_grasp_state")
            elif key == keyboard.KeyCode(char="5"):
                self.right_hand_grasp_state = True
                print("right_hand_grasp_state")
            elif key == keyboard.KeyCode(char="6"):
                self.start_signal = True
                self.run_signal = True
                print("start_signal + run_signal")
            elif key == keyboard.KeyCode(char="7"):
                self.damping_signal = True
                print("damping_signal")

            # Movement
            elif key == keyboard.KeyCode(char="w"):
                self.move_x = self.move_speed
                #print(f"move_x: {self.move_x}")
            elif key == keyboard.KeyCode(char="s"):
                self.move_x = -self.move_speed
                #print(f"move_x: {self.move_x}")
            elif key == keyboard.KeyCode(char="a"):
                self.move_y = self.move_speed
                #print(f"move_y: {self.move_y}")
            elif key == keyboard.KeyCode(char="d"):
                self.move_y = -self.move_speed
                #print(f"move_y: {self.move_y}")
            elif key == keyboard.Key.up:  
                self.turn_x = self.turn_speed
                #print(f"turn_x: {self.turn_x}")
            elif key == keyboard.Key.down:
                self.turn_x = -self.turn_speed
                #print(f"turn_x: {self.turn_x}")
            elif key == keyboard.Key.left:
                self.turn_y = self.turn_speed
                #print(f"turn_y: {self.turn_y}")
            elif key == keyboard.Key.right:
                self.turn_y = -self.turn_speed
                #print(f"turn_y: {self.turn_y}")
        except Exception as e:
            print(e)

    def on_release(self, key):
        try:
            # Signals
            if key == keyboard.KeyCode(char="1"):
                self.run_loco_signal = False
            elif key == keyboard.KeyCode(char="2"):
                self.run_squat_signal = False
            elif key == keyboard.KeyCode(char="3"):
                self.stopgait_signal = False
            elif key == keyboard.KeyCode(char="4"):
                self.left_hand_grasp_state = False
            elif key == keyboard.KeyCode(char="5"):
                self.right_hand_grasp_state = False
            elif key == keyboard.KeyCode(char="6"):
                self.start_signal = False
                self.run_signal = False
            elif key == keyboard.KeyCode(char="7"):
                self.damping_signal = False
            
            # Movement
            elif key == keyboard.KeyCode(char="w") or key == keyboard.KeyCode(char="s"):
                self.move_x = 0
            elif key == keyboard.KeyCode(char="a") or key == keyboard.KeyCode(char="d"):
                self.move_y = 0
            elif key == keyboard.Key.up or key == keyboard.Key.down:
                self.turn_x = 0
            elif key == keyboard.Key.left or key == keyboard.Key.right:
                self.turn_y = 0
            elif key == keyboard.Key.esc:
                self.stop_listener()
        except Exception as e:
            print(e)

