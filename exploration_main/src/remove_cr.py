# This script removes \r characters from a python file that caused issues when running it 
file = "/home/valeriofranchi/catkin_ws/src/exploration_main/src/main_exp.py"
with open(file, "rb+") as f:
    content = f.read()
    f.seek(0)
    f.write(content.replace(b'\r', b''))
    f.truncate()