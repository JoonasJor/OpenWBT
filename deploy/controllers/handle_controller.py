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
        self.speed = 2.0

    def start_receiving(self):
        self.running = True
        self.listener.start()

    def stop_receiving(self):
        self.running = False
        self.listener.stop()
            
    def on_press(self, key):
        try:
            if key.char == "1":
                self.run_loco_signal = True
                print(f"run_loco_signal: {self.run_loco_signal}")
            elif key.char == "2":
                self.run_squat_signal = True
                print(f"run_squat_signal: {self.run_squat_signal}")
            elif key.char == "3":
                self.stopgait_signal = True
                print(f"stopgait_signal: {self.stopgait_signal}")
            elif key.char == "4":
                self.left_hand_grasp_state = True
                print(f"left_hand_grasp_state: {self.left_hand_grasp_state}")
            elif key.char == "5":
                self.right_hand_grasp_state = True
                print(f"right_hand_grasp_state: {self.right_hand_grasp_state}")
            elif key.char == "6":
                self.start_signal = True
                print(f"start_signal: {self.start_signal}")
            elif key.char == "7":
                self.run_signal = True
                print(f"run_signal: {self.run_signal}")
            elif key.char == "8":
                self.damping_signal = True
                print(f"damping_signal: {self.damping_signal}")
            elif key.char == "w":
                self.move_x = self.speed
                print(f"move_x: {self.move_x}")
            elif key.char == "s":
                self.move_x = -self.speed
                print(f"move_x: {self.move_x}")
            elif key.char == "a":
                self.move_y = self.speed
                print(f"move_y: {self.move_y}")
            elif key.char == "d":
                self.move_y = -self.speed
                print(f"move_y: {self.move_y}")
        except Exception as e:
            pass

        try:
            if key == keyboard.Key.up:  
                self.turn_x = self.speed
                print(f"turn_x: {self.turn_x}")
            elif key == keyboard.Key.down:
                self.turn_x = -self.speed
                print(f"turn_x: {self.turn_x}")
            elif key == keyboard.Key.left:
                self.turn_y = self.speed
                print(f"turn_y: {self.turn_y}")
            elif key == keyboard.Key.right:
                self.turn_y = -self.speed
                print(f"turn_y: {self.turn_y}")
        except Exception as e:
            pass

    def on_release(self, key):
        try:
            if key.char == "1":
                self.run_loco_signal = False
            elif key.char == "4":
                self.run_squat_signal = False
            elif key.char == "2":
                self.stopgait_signal = False
            elif key.char == "3":
                self.left_hand_grasp_state = False
            elif key.char == "6":
                self.right_hand_grasp_state =False
            elif key.char == "5":
                self.start_signal = False
            elif key.char == "7":
                self.run_signal = False
            elif key.char == "8":
                self.damping_signal = False
            elif key.char == "w" or key.char == "s":
                self.move_x = 0
            elif key.char == "a" or key.char == "d":
                self.move_y = 0
        except Exception:
            pass
        
        try:
            if key == keyboard.Key.up or key == keyboard.Key.down:
                self.turn_x = 0
            elif key == keyboard.Key.left or key == keyboard.Key.right:
                self.turn_y = 0
            elif key == keyboard.Key.esc:
                print("Stopped keyboard listener")
                self.stop_receiving()
        except Exception as e:
            pass

