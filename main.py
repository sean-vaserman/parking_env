import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider, Button
import numpy as np

from kinematics import VehicleModel
from stanley_controller import StanleyController
from hybrid_a_star import plan_path

# -----------------------------------------------------------------------
# Layout (metres, origin bottom-left)
#
#  y = 0  … 7.0   ← road (7 m wide east-west street)
#  y = 7.0 … 13.0  ← driveway (6 m long, same width as garage)
#  y = 13.0 … 19.5 ← garage interior (6.5 m deep, 3.7 m wide)
#  y = 19.5         ← back wall
#
# The horizontal curbs at y=7 have a 4.3 m gap (x=9.35–13.65) for
# the driveway entrance.  Channel walls run continuously from the curb
# level all the way to the garage back wall, so the only traversable
# path from road to garage is through that gap.
# -----------------------------------------------------------------------

ROAD_CURBS = [
    (0.0,   7.0,  9.35, 0.3),   # left  road curb (blocks grass access)
    (13.65, 7.0, 10.35, 0.3),   # right road curb
]
CHANNEL_WALLS = [
    (9.35,  7.0, 0.3, 12.5),    # left  wall: driveway + garage (y=7 → 19.5)
    (13.35, 7.0, 0.3, 12.5),    # right wall
]
GARAGE_BACK = [
    (9.35, 19.5, 4.3, 0.3),     # back wall
]
GARAGE_OBJECTS = [
    (9.65, 16.5, 0.4, 1.5),     # shelf  (left side of garage)
    (12.9, 13.5, 0.5, 0.6),     # bin    (right front corner)
]
PARKED_CARS = [
    (0.5,  5.0, 3.0, 2.0),      # left  parked car (x=0.5–3.5, y=2–4)
    (20.5, 5.0, 3.0, 2.0),      # right parked car (x=20.5–23.5, y=2–4)
]

OBSTACLES = ROAD_CURBS + CHANNEL_WALLS + GARAGE_BACK + GARAGE_OBJECTS + PARKED_CARS

# Waypoint-advancement parameters
ADVANCE_DIST = 0.8   # m: advance to next waypoint when within this distance
BRAKE_DIST   = 0.7   # m: start braking before a gear-change waypoint
STOP_V       = 0.2   # m/s: speed threshold to cross a gear boundary


def draw_environment(ax):
    ax.clear()
    ax.set_xlim(0, 24)
    ax.set_ylim(0, 20)
    ax.set_aspect('equal')
    ax.set_title("Autonomous Garage Parking PoC")

    # Road surface
    ax.add_patch(patches.Rectangle((0, 0), 24, 7.0, color='#b0b0b0'))
    # Driveway surface (between curbs and garage entrance)
    ax.add_patch(patches.Rectangle((9.65, 7.0), 3.7, 6.0, color='#c4c4c4'))
    # Garage interior floor
    ax.add_patch(patches.Rectangle((9.65, 13.0), 3.7, 6.5, color='#dcdcdc'))
    # Grass — left and right of the driveway/garage channel
    ax.add_patch(patches.Rectangle((0.0,   7.0),  9.35, 13.0, color='#6ab56a'))
    ax.add_patch(patches.Rectangle((13.65, 7.0), 10.35, 13.0, color='#6ab56a'))

    # Road centre line — east-west dashes at y=3.5
    for x in np.arange(0.5, 23.5, 2.5):
        ax.plot([x, x + 1.5], [3.5, 3.5], color='#f0f040', linewidth=2)

    # Road curbs (physical obstacles — drawn in dark grey)
    for (x, y, w, h) in ROAD_CURBS:
        ax.add_patch(patches.Rectangle((x, y), w, h, color='#505050'))

    # Channel walls and back wall
    for (x, y, w, h) in CHANNEL_WALLS + GARAGE_BACK:
        ax.add_patch(patches.Rectangle((x, y), w, h, color='#282828'))

    # Garage objects
    for (x, y, w, h) in GARAGE_OBJECTS:
        ax.add_patch(patches.Rectangle((x, y), w, h, color='#8B4513'))

    # Parked cars
    for (x, y, w, h) in PARKED_CARS:
        ax.add_patch(patches.Rectangle((x, y), w, h, color='#1a3a6c'))

    # Labels
    ax.text(10.0, 9.5,  'DRIVEWAY', fontsize=6,  color='#555555',
            rotation=90, va='bottom', ha='center')
    ax.text(10.0, 15.5, 'GARAGE',   fontsize=7,  color='#777777',
            rotation=90, va='center', ha='center')


def draw_vehicle(ax, vehicle, steer=0.0):
    outline = np.array([
        [-vehicle.L / 2, vehicle.L / 2, vehicle.L / 2, -vehicle.L / 2, -vehicle.L / 2],
        [ vehicle.W / 2, vehicle.W / 2, -vehicle.W / 2, -vehicle.W / 2,  vehicle.W / 2]
    ])
    rot = np.array([
        [np.cos(vehicle.yaw), -np.sin(vehicle.yaw)],
        [np.sin(vehicle.yaw),  np.cos(vehicle.yaw)]
    ])
    outline = (rot @ outline).T
    outline[:, 0] += vehicle.x
    outline[:, 1] += vehicle.y
    ax.plot(outline[:, 0], outline[:, 1], 'b-', linewidth=1.5)

    wheel_len = 0.6
    front_axle_x = vehicle.x + (vehicle.L / 2) * np.cos(vehicle.yaw)
    front_axle_y = vehicle.y + (vehicle.L / 2) * np.sin(vehicle.yaw)
    lw_x = front_axle_x - (vehicle.W / 2) * np.sin(vehicle.yaw)
    lw_y = front_axle_y + (vehicle.W / 2) * np.cos(vehicle.yaw)
    rw_x = front_axle_x + (vehicle.W / 2) * np.sin(vehicle.yaw)
    rw_y = front_axle_y - (vehicle.W / 2) * np.cos(vehicle.yaw)
    wheel_yaw = vehicle.yaw + steer
    for wx, wy in [(lw_x, lw_y), (rw_x, rw_y)]:
        wx1 = wx - (wheel_len / 2) * np.cos(wheel_yaw)
        wy1 = wy - (wheel_len / 2) * np.sin(wheel_yaw)
        wx2 = wx + (wheel_len / 2) * np.cos(wheel_yaw)
        wy2 = wy + (wheel_len / 2) * np.sin(wheel_yaw)
        ax.plot([wx1, wx2], [wy1, wy2], 'r-', linewidth=3)


def advance_target(vehicle, px, py, pgear, target_idx):
    """Advance using rear-axle distance; never cross a gear boundary while moving."""
    if target_idx >= len(px) - 1:
        return target_idx
    dist = math.hypot(vehicle.x - px[target_idx], vehicle.y - py[target_idx])
    if pgear[target_idx + 1] != pgear[target_idx]:
        if dist < ADVANCE_DIST and abs(vehicle.v) < STOP_V:
            return target_idx + 1
    else:
        if dist < ADVANCE_DIST:
            return target_idx + 1
    return target_idx


class SimulationApp:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 9))
        plt.subplots_adjust(bottom=0.35)

        # Goal: centre of garage, pointing north
        self.goal_pose = (11.5, 15.5, np.pi / 2)
        self.is_running = False

        axcolor = 'lightgoldenrodyellow'
        self.ax_x   = plt.axes([0.15, 0.20, 0.65, 0.03], facecolor=axcolor)
        self.ax_y   = plt.axes([0.15, 0.15, 0.65, 0.03], facecolor=axcolor)
        self.ax_yaw = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor=axcolor)

        # X: 5–19 keeps start well clear of parked cars (which end at x=3.5 / start at x=20.5)
        # Y: 1.5–5.9 keeps start ≥ 1.05 m from road curb at y=7
        self.slider_x   = Slider(self.ax_x,   'Start X',  5.0, 19.0, valinit=7.0)
        self.slider_y   = Slider(self.ax_y,   'Start Y',  1.5,  5.9, valinit=4.0)
        self.slider_yaw = Slider(self.ax_yaw, 'Heading', -np.pi, np.pi, valinit=np.radians(45))

        self.ax_btn = plt.axes([0.3, 0.02, 0.2, 0.05])
        self.btn_run = Button(self.ax_btn, 'Plan & Drive', hovercolor='0.975')
        self.btn_run.on_clicked(self.run_simulation)

        self.ax_btn_reset = plt.axes([0.55, 0.02, 0.2, 0.05])
        self.btn_reset = Button(self.ax_btn_reset, 'Reset', hovercolor='0.975')
        self.btn_reset.on_clicked(self.reset_simulation)

        self.update_preview(None)
        self.slider_x.on_changed(self.update_preview)
        self.slider_y.on_changed(self.update_preview)
        self.slider_yaw.on_changed(self.update_preview)

        plt.show()

    def update_preview(self, _val):
        if not self.is_running:
            draw_environment(self.ax)
            dummy = VehicleModel(x=self.slider_x.val, y=self.slider_y.val,
                                 yaw=self.slider_yaw.val)
            draw_vehicle(self.ax, dummy)
            self.fig.canvas.draw_idle()

    def reset_simulation(self, _event):
        self.is_running = False
        self.update_preview(None)

    def run_simulation(self, _event):
        if self.is_running:
            return
        self.is_running = True

        start_pose = (self.slider_x.val, self.slider_y.val, self.slider_yaw.val)
        vehicle    = VehicleModel(x=start_pose[0], y=start_pose[1], yaw=start_pose[2])
        controller = StanleyController(k=1.2)

        px, py, pyaw, pgear = plan_path(start_pose, self.goal_pose, OBSTACLES, vehicle)
        if not px:
            print("No valid path found from this location.")
            self.is_running = False
            return

        dt        = 0.1
        target_idx = 0
        last_idx   = len(px) - 1
        # Stays True once the car enters BRAKE_DIST for a gear-change;
        # prevents the overshoot case from re-engaging the wrong gear.
        braking_for_gear_change = False

        for _ in range(2000):
            if not self.is_running:
                break

            # --- Waypoint advancement ---
            prev_idx   = target_idx
            target_idx = advance_target(vehicle, px, py, pgear, target_idx)
            if target_idx != prev_idx:
                braking_for_gear_change = False

            # Force advance when stopped during a gear-change brake
            if braking_for_gear_change and abs(vehicle.v) < STOP_V and target_idx < last_idx:
                target_idx += 1
                braking_for_gear_change = False

            gear  = pgear[target_idx]
            steer = controller.control(vehicle, px, py, pyaw, pgear, target_idx)

            dist_to_target    = math.hypot(vehicle.x - px[target_idx], vehicle.y - py[target_idx])
            next_is_gear_change = (target_idx < last_idx and pgear[target_idx + 1] != gear)

            # --- Desired velocity ---
            if target_idx >= last_idx:
                desired_v = 0.0
            elif next_is_gear_change:
                if dist_to_target < BRAKE_DIST:
                    braking_for_gear_change = True
                desired_v = 0.0 if braking_for_gear_change else 2.0 * gear
            else:
                braking_for_gear_change = False
                desired_v = 2.0 * gear

            # --- Throttle ---
            v_err = desired_v - vehicle.v
            if desired_v * vehicle.v < 0 and abs(vehicle.v) > 0.5:
                throttle = 3.0 * v_err     # aggressive direction-change braking
            else:
                throttle = 1.0 * v_err
            throttle = np.clip(throttle, -3.0, 2.0)

            # --- Arrival ---
            if desired_v == 0.0 and abs(vehicle.v) < 0.1 and target_idx >= last_idx:
                steer    = 0.0
                throttle = 0.0
                draw_environment(self.ax)
                self.ax.plot(px, py, 'g--', linewidth=1)
                draw_vehicle(self.ax, vehicle, steer)
                plt.pause(0.01)
                print("Successfully Parked and Aligned!")
                break

            vehicle.update(throttle, steer, dt)

            # --- Render ---
            draw_environment(self.ax)
            self.ax.plot(px, py, 'g--', linewidth=1)
            self.ax.plot(px[target_idx], py[target_idx], 'go', markersize=5)
            draw_vehicle(self.ax, vehicle, steer)
            plt.pause(0.01)

        self.is_running = False


if __name__ == '__main__':
    SimulationApp()
