import numpy as np
import os
import time
import serial
import random

NUM_STATE = 3
NUM_ACTION = 2
NUM_POLE_ANGLE_BINS = 120
NUM_POLE_V_BINS = 10
NUM_CART_POS_BINS = 10

POLE_ANGLE_MIN = 0
POLE_ANGLE_MAX = 360
POLE_VELOCITY_MIN = -100
POLE_VELOCITY_MAX = 100
CART_POSITION_MIN = -400
CART_POSITION_MAX = 400

DELAY = 0.05

ALPHA = 0.1 # Learning Rate
GAMMA = 0.9 # Discount Factor
EPSILON = 0.1 # Exploration Rate

Q_TABLE_FILE = 'Q_table.npy'
ITERATION_COUNT_FILE = 'Iteration_Count.txt'

Q_table = None
State = None
State_New = None
Action = None
PrintMode = True
Done = False

ser = serial.Serial('COM5', 9600)


def save_Q_table():
    np.save(Q_TABLE_FILE, Q_table)

def load_Q_table():
    try:
        Q_table = np.load(Q_TABLE_FILE)
        print("Q Table successfully loaded")
    except FileNotFoundError:
        Q_table = init_Q_table()
        print("Q Table does not exist, will initialize Q Table")
    return Q_table

def read_iteration_count():
    if os.path.exists(ITERATION_COUNT_FILE):
        with open(ITERATION_COUNT_FILE, 'r') as file:
            return int(file.read().strip())
    return 0

def write_iteration_count(count):
    with open(ITERATION_COUNT_FILE, 'w') as file:
        file.write(str(count))

def iteration_count():
    count = read_iteration_count()
    count = count+1
    write_iteration_count(count)
    print(f"Current training rounds: {count}")

def init_Q_table():
    num_states = NUM_POLE_ANGLE_BINS * NUM_POLE_V_BINS * NUM_CART_POS_BINS
    Q_table = np.zeros((num_states, NUM_ACTION))
    return Q_table

def discretize(value, min_value, max_value, bins):
    value = np.clip(value, min_value, max_value - 1e-5)
    bin_size = (max_value - min_value) / bins
    return int((value - min_value) / bin_size)

def state_to_index(state):
    pole_angle, pole_velocity, cart_position = state
    pole_angle_idx = discretize(pole_angle, POLE_ANGLE_MIN, POLE_ANGLE_MAX, NUM_POLE_ANGLE_BINS)
    pole_velocity_idx = discretize(pole_velocity, POLE_VELOCITY_MIN, POLE_VELOCITY_MAX, NUM_POLE_V_BINS)
    cart_position_idx = discretize(cart_position, CART_POSITION_MIN, CART_POSITION_MAX, NUM_CART_POS_BINS)
    index = pole_angle_idx * (NUM_POLE_V_BINS * NUM_CART_POS_BINS) + pole_velocity_idx * NUM_CART_POS_BINS + cart_position_idx
    if PrintMode:
        print(f"The index of this state is :{index}")
    return index

def print_whole_Q_table():
    original_printoptions = np.get_printoptions()
    np.set_printoptions(threshold=np.inf)
    print(Q_table)
    np.set_printoptions(**original_printoptions)

def print_Q_table(begin, end):
    print(Q_table[begin:end])
    
def get_Q_line(pole_angle, pole_velocity, cart_position):
    index = state_to_index(pole_angle, pole_velocity, cart_position)
    print(Q_table[index])

def get_Q_value(pole_angle, pole_velocity, cart_position, action):
    index = state_to_index(pole_angle, pole_velocity, cart_position)
    print(Q_table[index, action])

def reward(state):
    pole_angle, pole_velocity, cart_position = state

    angle_weight = 0.6
    velocity_weight = 0.1
    position_weight = 0.3

    angle_reward = 0
    velocity_reward = 0
    position_reward = 0

    if 175 <= pole_angle <= 185:
        angle_reward = 1
    elif pole_angle < 30 or pole_angle > 330:
        angle_reward = -10000

    if abs(pole_velocity) < 1:
        velocity_reward = 1

    if abs(cart_position) < 50:
        position_reward = 1
    elif abs(cart_position) > 400:
        position_reward = -10000

    total_reward = (angle_weight * angle_reward +
                    velocity_weight * velocity_reward +
                    position_weight * position_reward)
    if PrintMode:
        print(f"The reward of this state is: {total_reward}")
    return total_reward

def get_random_state():
    pole_angle = np.random.uniform(POLE_ANGLE_MIN, POLE_ANGLE_MAX)
    pole_velocity = np.random.uniform(POLE_VELOCITY_MIN, POLE_VELOCITY_MAX)
    cart_position = np.random.uniform(CART_POSITION_MIN, CART_POSITION_MAX)
    state = np.array([pole_angle, pole_velocity, cart_position])
    if PrintMode:
        print(f"Random state is generated: \nangle = {pole_angle}\nv = {pole_velocity}\npos = {cart_position}\n")
    return state

def get_max_Q_value(state):

    index = state_to_index(state)
    max_value = np.max(Q_table[index, :])
    if PrintMode:
        print(f"Print the line with index {index}...")
        print(Q_table[index])
        print(f"The max value is {max_value}")
    return max_value

def update_Q_table(s, s_new, a):
    print("updating Q Table...")
    index = state_to_index(s)
    r = reward(s)
    Q_table[index,a] = Q_table[index,a] + ALPHA * (r + GAMMA * get_max_Q_value(s_new) -Q_table[index,a])
    print(f"The updated Q Value of Q_table[{index},{a}] is {Q_table[index,a]}")


def check_done():
    state = get_state(0)
    if abs(state[0] - 180) > 45:
        print("Pole angle out of range, terminated")
        return True
    if abs(state[2]) > CART_POSITION_MAX:
        print("Cart position out of range, terminated")
        return True
    
    return False

def wait_for_button_press():
    print("Please press the button to start a new round of training...")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line == "Switch pressed, loop on":
                print("Button pressed, training begins...")
                break
        time.sleep(DELAY)

def wait_for_motor_move():
    print("Waiting for Motor move...")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line == "Motor moved":
                break
        time.sleep(DELAY)

def get_state(par):
    state = (180, 0, 0)
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        if par == 2:
            print(line)
        if line.startswith("States:"):
            state = parse_state_string(line)
            if state:
                pole_angle, pole_v, cart_pos = state
                if par == 1:
                    print(f"Angle: {pole_angle}, Velocity: {pole_v}, Displacement: {cart_pos}")

    return state

def parse_state_string(state_string):
    parts = state_string.split()
    if len(parts) == 4 and parts[0] == "States:":
        try:
            int_values = [int(parts[1]), int(parts[2]), int(parts[3])]
            return int_values
        except ValueError:
            return None
    return None

Q_table = load_Q_table()
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)

    wait_for_button_press()
    iteration_count()

    while not Done:
        State = get_state(1)
        if abs(State[0] - 180) > 90:
            print("Pole angle out of range, terminated")
            save_Q_table()
            break
        if abs(State[2]) > 350:
            print("Cart position out of range, terminated")
            save_Q_table()
            break
        if random.uniform(0, 1) < EPSILON:
            Action = random.choice(range(NUM_ACTION))
            print(f"Random Action: {Action}")
        else:
            Action = np.argmax(Q_table[state_to_index(State), :])
            print(f"Q Table Action: {Action}")
        
        ser.write(f"Action: {Action}\n".encode())
        wait_for_motor_move()
        State_New = get_state(1)
        update_Q_table(State,State_New,Action)
        State = State_New

        

        time.sleep(0.05)
    print("End of the training round")
