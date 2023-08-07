import bisect
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
import threading
from rosplane2_msgs.msg import State
from rosflight_msgs.msg import Command
from rosplane2_msgs.msg import ControllerInternals
from rosplane2_msgs.msg import ControllerCommands


def trim_data(time: list[float], data: list[float], val_thresh: float) -> None:
    """Trims the time and data vectors to only have time values over the latest 'val_thresh' window"""

    ind = bisect.bisect_left(time, time[-1] - val_thresh)
    if ind > 0:
        del data[0:ind]
        del time[0:ind]

def time_to_seconds(time: TimeMsg) -> float:
    """Converts the time to seconds
    """
    seconds = time.sec + time.nanosec*1.e-9
    return seconds

class StateStorage:
    """Stores state data created from UavState"""

    def __init__(self, t_horizon) -> None:
        """ Initialize the state storage

        Inputs:
            t_horizon: Time horizon for storage
        """

        # Store variables
        self.t_horizon = t_horizon      # Time horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.pn: list[float] = []       # North positions
        self.pe: list[float] = []       # East positions
        self.p_alt: list[float] = []    # Altitude
        self.v_a: list[float] = []      # Airspeed
        self.alpha: list[float] = []    # Angle of attack
        self.beta: list[float] = []     # sideslip angle
        self.phi: list[float] = []      # Attitude euler angles
        self.theta: list[float] = []
        self.psi: list[float] = []
        self.chi: list[float] = []      # course angle
        self.u: list[float] = []        # x,y,z body frame velocities
        self.v: list[float] = []
        self.w: list[float] = []
        self.p: list[float] = []        # x,y,z body frame rotational velocities
        self.q: list[float] = []
        self.r: list[float] = []
        self.v_g: list[float] = []      # Ground speed
        self.w_n: list[float] = []      # North wind
        self.w_e: list[float] = []      # East wind


    def append(self, state: State):
        """ Stores the state data and trims the vectors
        """

        if(state.header.stamp.sec == 0):
            return

        # Store the data
        self.time.append(time_to_seconds(state.header.stamp))
        self.pn.append(state.position.item(0))
        self.pe.append(state.position.item(1))
        self.p_alt.append(-state.position.item(2))  # -1 to get the altitude instead of down
        self.phi.append(state.phi)
        self.theta.append(state.theta)
        self.psi.append(state.psi)
        self.u.append(state.u)
        self.v.append(state.v)
        self.w.append(state.w)
        self.p.append(state.p)
        self.q.append(state.q)
        self.r.append(state.r)
        self.w_n.append(state.wn)
        self.w_e.append(state.we)
        self.v_a.append(state.va)
        self.v_g.append(state.vg)
        self.alpha.append(state.alpha)
        self.beta.append(state.beta)
        self.chi.append(state.chi)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.pn[0:ind]
            del self.pe[0:ind]
            del self.p_alt[0:ind]
            del self.phi[0:ind]
            del self.theta[0:ind]
            del self.psi[0:ind]
            del self.u[0:ind]
            del self.v[0:ind]
            del self.w[0:ind]
            del self.p[0:ind]
            del self.q[0:ind]
            del self.r[0:ind]
            del self.w_n[0:ind]
            del self.w_e[0:ind]
            del self.v_a[0:ind]
            del self.v_g[0:ind]
            del self.alpha[0:ind]
            del self.beta[0:ind]
            del self.chi[0:ind]

class CommandStorage:
    """Stores the vehicle commands
    """
    def __init__(self, t_horizon) -> None:
        """ Initialize the command storage
        """
        # Store the horizon variable
        self.t_horizon = t_horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.elevator: list[float] = []
        self.aileron: list[float] = []
        self.rudder: list[float] = []
        self.throttle: list[float] = []

    def append(self, cmd: Command) -> None:
        """ Stores the command data and trims the vectors
        """
        # Append data
        self.time.append(time_to_seconds(cmd.header.stamp))
        self.elevator.append(cmd.y)
        self.aileron.append(cmd.x)
        self.rudder.append(cmd.z)
        self.throttle.append(cmd.f)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.elevator[0:ind]
            del self.aileron[0:ind]
            del self.rudder[0:ind]
            del self.throttle[0:ind]

class ControllerInternalsStorage:

    def __init__(self, t_horizon: float) -> None:

        self.t_horizon = t_horizon

        self.time: list[float] = []     # Time values (seconds)
        self.pitch_c: list[float] = []
        self.roll_c: list[float] = []

    def append(self, internals: ControllerInternals) -> None:
        """ Stores the command data and trims the vectors
        """
        # Append data
        self.time.append(time_to_seconds(internals.header.stamp))
        self.pitch_c.append(internals.theta_c)
        self.roll_c.append(internals.phi_c)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.pitch_c[0:ind]
            del self.roll_c[0:ind]

class ControllerCommandsStorage:

    def __init__(self, t_horizon: float) -> None:

        self.t_horizon = t_horizon

        self.time: list[float] = []     # Time values (seconds)
        self.course_c: list[float] = []
        self.alt_c: list[float] = []
        self.airspeed_c: list[float] = []

    def append(self, con_cmd: ControllerCommands) -> None:
        """ Stores the command data and trims the vectors
        """
        # Append data
        self.time.append(time_to_seconds(con_cmd.header.stamp))
        self.course_c.append(con_cmd.chi_c)
        self.alt_c.append(con_cmd.h_c)
        self.airspeed_c.append(con_cmd.va_c)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.course_c[0:ind]
            del self.alt_c[0:ind]
            del self.airspeed_c[0:ind]


class RosStorageInterface:
    """ Uses a given node to subscribe to state, command, and sensory information
    """

    def __init__(self, node: Node, t_horizon: float) -> None:
        """ Create the ros interfaces required for storage

        Inputs:
            node: Node to use for subscriptions
            t_horizon: Time horizon of data to keep
        """

        # Store inputs
        self.node = node
        self.t_horizon = t_horizon

        # Initialize the state parameters
        self.t_horizon = 100. # TODO Read in instead of hard code
        self.lock = threading.Lock() # The lock is used to allow data to not be received / updated
        # while being accessed


        # Initialize the ros variables
        self._sub_state = self.node.create_subscription(State, "/state", self.state_callback, 1)
        self._sub_est = self.node.create_subscription(State, "/estimated_state", self.estimate_callback, 1)
        self._sub_cmd = self.node.create_subscription(Command, "/command", self.command_callback, 1)
        self._sub_cmd_internals = self.node.create_subscription(ControllerInternals, "/controller_inners", self.cmd_internal_callback, 1)
        self._sub_con_cmd = self.node.create_subscription(ControllerCommands, "/controller_commands", self.con_command_callback, 1)

        self.con_cmd = ControllerCommands()

        # Initailize the storage
        self.true = StateStorage(t_horizon=self.t_horizon)
        self.cmd = CommandStorage(t_horizon=self.t_horizon)
        self.est = StateStorage(t_horizon=self.t_horizon)
        self.con_inners = ControllerInternalsStorage(t_horizon=self.t_horizon)
        self.con_cmd = ControllerCommandsStorage(t_horizon=self.t_horizon)

    def state_callback(self, msg: State) -> None:
        """Stores the latest state data
        """
        with self.lock:
            self.true.append(state=msg)

    def estimate_callback(self, msg: State) -> None:
        """Stores the latest estimated state data
        """
        # self.node.get_logger().info("In estimated state callback.")
        with self.lock:
            self.est.append(state=msg)


    def command_callback(self, msg: Command) -> None:
        """Stores the latest command data
        """
        with self.lock:
            self.cmd.append(cmd=msg)


    def cmd_internal_callback(self, msg: ControllerInternals) -> None:

        with self.lock:
            self.con_inners.append(internals=msg)

    def con_command_callback(self, msg: ControllerCommands) -> None:

        with self.lock:
            self.con_cmd.append(con_cmd=msg)