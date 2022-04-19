##from pynput import keyboard
##
##def on_press(key):
##    if(key == keyboard.Key.up):
##        print("up")
##    elif(key == keyboard.Key.down):
##        print("down")
##    elif(key == keyboard.Key.left):
##        print("left")
##    elif(key == keyboard.Key.right):
##        print("right")
##
##    elif(key == keyboard.Key.enter):
##        listener.stop()
##        
##
##def on_release(key):
##    if(key == keyboard.Key.up or key == keyboard.Key.down or key == keyboard.Key.left or key == keyboard.Key.right):
##        print("STOP")
##
##with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
##    listener.join()
##print("a")
##
###listener = keyboard.Listener(on_press=on_press, on_release=on_release)
##
##print("b")
##
###listener.start()
##print("Bye bye")
import keyboard

while True:
    if keyboard.read_key() == "p":
        print("You pressed p")
        break

while True:
    if keyboard.is_pressed("q"):
        print("You pressed q")
        break
        
#keyboard.on_press_key("r", lambda _:print("You pressed r"))
