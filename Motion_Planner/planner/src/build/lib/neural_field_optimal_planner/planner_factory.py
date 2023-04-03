import torch
from pytorch_lightning.utilities.parsing import AttributeDict

from .astar.astar_trajectory_initializer import AstarTrajectoryInitializer
from .constrained_nerf_opt_planner import ConstrainedNERFOptPlanner
from .nerf_opt_planner import NERFOptPlanner
from .onf_model import ONF
from .trajectory_initializer import TrajectoryInitializer
from .utils.universal_factory import UniversalFactory

DEFAULT_PARAMETERS = AttributeDict(
    device="cpu",
    trajectory_length=100,
    collision_model=AttributeDict(
        mean=0,
        sigma=10,
        use_cos=True,
        bias=True,
        use_normal_init=True,
        name="ONF"
    ),
    collision_optimizer=AttributeDict(
        lr=1e-2,
        betas=(0.9, 0.9)
    ),
    trajectory_optimizer=AttributeDict(
        lr=1e-2,
        betas=(0.9, 0.9)
    ),
    planner=AttributeDict(
        name="ConstrainedNERFOptPlanner",
        trajectory_random_offset=0.02,
        collision_weight=1,
        velocity_hessian_weight=0.5,
        random_field_points=10,
        init_collision_iteration=0,
        constraint_deltas_weight=0.2,
        multipliers_lr=0.001,
        init_collision_points=100,
        reparametrize_trajectory_freq=10,
        optimize_collision_model_freq=1,
        angle_weight=0.5,
        boundary_weight=1,
        collision_multipliers_lr=1e-3
    )
)


class PlannerFactory(object):
    @staticmethod
    def make_onf_planner(collision_checker):
        device = "cpu"
        collision_model = ONF(1.5, 1).to(device)
        collision_optimizer = torch.optim.Adam(collision_model.parameters(), 1e-3, betas=(0.9, 0.9))
        trajectory = torch.zeros(100, 2, requires_grad=True, device=device)
        trajectory_optimizer = torch.optim.Adam([trajectory], 1e-2, betas=(0.9, 0.999))
        return NERFOptPlanner(trajectory, collision_model, collision_checker, collision_optimizer,
                              trajectory_optimizer, trajectory_random_offset=0.02, collision_weight=0.01,
                              velocity_hessian_weight=3, random_field_points=10, init_collision_iteration=400)

    @staticmethod
    def make_constrained_onf_planner(collision_checker, parameters=None):
        if parameters is None:
            parameters = DEFAULT_PARAMETERS
        factory = UniversalFactory([ONF, ConstrainedNERFOptPlanner, TrajectoryInitializer, AstarTrajectoryInitializer])
        device = parameters.device
        collision_model = factory.make_from_parameters(parameters.collision_model).to(device)
        collision_optimizer = torch.optim.Adam(collision_model.parameters(), **parameters.collision_optimizer)
        trajectory = torch.zeros(parameters.trajectory_length, 3, requires_grad=True, device=device)
        trajectory_optimizer = torch.optim.Adam([trajectory], **parameters.trajectory_optimizer)
        trajectory_initializer = factory.make_from_parameters(parameters.trajectory_initializer,
                                                              collision_checker=collision_checker)
        return factory.make_from_parameters(parameters.planner, trajectory=trajectory, collision_model=collision_model,
                                            collision_checker=collision_checker,
                                            collision_optimizer=collision_optimizer,
                                            trajectory_optimizer=trajectory_optimizer,
                                            trajectory_initializer=trajectory_initializer)
