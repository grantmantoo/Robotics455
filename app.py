from flask import Flask, request, jsonify, render_template
from billy_controller import Billy
app = Flask(__name__, static_url_path='/static', template_folder="Templates")
billy = Billy()
billy.reset()
@app.route('/')
def index():
    return render_template('index.html')
@app.route('/control', methods=['POST'])
def control_robot():
    data = request.form
    joystick_x = 3*float(data.get('joystickX'))
    joystick_y = 3*float(data.get('joystickY'))
    button = int(data.get('button'))
    #Move_Backward Logic
    if(joystick_y > 0 and joystick_y <= 1):
        billy.move_backward(speed = joystick_y)
    if(joystick_y > 1 and joystick_y <= 2):
        billy.move_backward(speed = joystick_y)
    if(joystick_y > 2 and joystick_y <= 3):
        billy.move_backward(speed = joystick_y)
    #Move_Forward Logic
    if(joystick_y < 0 and joystick_y >= -1):
        billy.move_forward(speed = -joystick_y)
    if(joystick_y < -1 and joystick_y >= -2):
        billy.move_forward(speed = -joystick_y)
    if(joystick_y < -2 and joystick_y >= -3):
        billy.move_forward(speed = -joystick_y)
    #Left/ Right Logic
    if(joystick_x < -1) and (joystick_y) < 1 and joystick_y > -1:
        billy.turn_left()
    if(joystick_x > 1) and (joystick_y) < 1 and joystick_y > -1:
        billy.turn_right()
    if(joystick_x == 0 and joystick_y == 0):
        billy.stop_motors()
    #Reset Button
    if(button == 1):
        billy.reset()
    return jsonify(success=True)
@app.route("/control_slider", methods=["POST"])
def control_sliders():
    data = request.form
    waist_position = int(data.get('waistPosition'))
    head_vertical = int(data.get('headVertical'))
    head_horizontal = int(data.get('headHorizontal'))
    #Head / Waist Logic
    billy.set_waist(position=waist_position)
    billy.set_head(nod = head_vertical, shake = head_horizontal)
    return jsonify(success=True)
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=2000)
