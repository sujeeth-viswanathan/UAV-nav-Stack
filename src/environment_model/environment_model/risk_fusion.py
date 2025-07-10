import numpy as np
from scipy.ndimage import uniform_filter

# ----------------------
# Configuration
# ----------------------
config = {
    "target_heading_rad": np.pi / 2,
    "wind_alpha": 0.4,
    "wind_beta": 0.3,
    "wind_gamma": 0.3,
    "wind_kernel_size": 3,
    "rain_max_safe_intensity": 5.0,
    "fog_max_safe_density": 0.7,
    "lightning_max_density": 5.0,
    "fusion_weights": {
        'wind': 0.4,
        'fog': 0.2,
        'lightning': 0.3,
        'rain': 0.1
    }
}

# ----------------------
# Wind Analysis Helpers
# ----------------------
def compute_wind_magnitude(wind_field):
    return np.sqrt(wind_field[..., 0] ** 2 + wind_field[..., 1] ** 2)

def compute_wind_direction(wind_field):
    return np.arctan2(wind_field[..., 1], wind_field[..., 0]) % (2 * np.pi)

def compute_local_std(field, kernel_size=3):
    k = config["wind_kernel_size"] if kernel_size is None else kernel_size
    mean = uniform_filter(field, size=k)
    sq_diff = uniform_filter(field ** 2, size=k) - mean ** 2
    return np.sqrt(sq_diff)

def compute_directional_variability(wind_field, kernel_size=None):
    angle = compute_wind_direction(wind_field)
    x = np.cos(angle)
    y = np.sin(angle)
    std_x = compute_local_std(x, kernel_size)
    std_y = compute_local_std(y, kernel_size)
    return np.sqrt(std_x ** 2 + std_y ** 2)

def compute_alignment_risk(wind_field, target_heading_rad):
    th = config["target_heading_rad"] if target_heading_rad is None else target_heading_rad
    wind_direction = compute_wind_direction(wind_field)
    angle_diff = np.abs(wind_direction - th)
    angle_diff = np.minimum(angle_diff, 2 * np.pi - angle_diff)
    return angle_diff / np.pi

# ----------------------
# Influence Models
# ----------------------
class WindInfluenceModel:
    def __init__(self, wind_field):
        self.wind_field = wind_field
        self.target_heading_rad = config["target_heading_rad"]
        self.alpha = config["wind_alpha"]
        self.beta = config["wind_beta"]
        self.gamma = config["wind_gamma"]
        self.kernel_size = config["wind_kernel_size"]

    def compute_risk(self):
        alignment_risk = compute_alignment_risk(self.wind_field, self.target_heading_rad)
        turbulence_index = compute_local_std(compute_wind_magnitude(self.wind_field), self.kernel_size)
        direction_var_index = compute_directional_variability(self.wind_field, self.kernel_size)

        t_norm = (turbulence_index - np.min(turbulence_index)) / (np.ptp(turbulence_index) + 1e-6)
        d_norm = (direction_var_index - np.min(direction_var_index)) / (np.ptp(direction_var_index) + 1e-6)

        wind_risk = self.alpha * alignment_risk + self.beta * t_norm + self.gamma * d_norm
        return np.clip(wind_risk, 0, 1)

class RainInfluenceModel:
    def __init__(self, rain_intensity_map):
        self.rain_intensity_map = rain_intensity_map
        self.max_safe_intensity = config["rain_max_safe_intensity"]

    def compute_risk(self):
        return np.clip(self.rain_intensity_map / self.max_safe_intensity, 0, 1)

class FogInfluenceModel:
    def __init__(self, fog_density_map):
        self.fog_density_map = fog_density_map
        self.max_safe_density = config["fog_max_safe_density"]

    def compute_risk(self):
        return np.clip(self.fog_density_map / self.max_safe_density, 0, 1)

class LightningInfluenceModel:
    def __init__(self, lightning_density_map):
        self.lightning_density_map = lightning_density_map
        self.max_safe_density = config["lightning_max_density"]

    def compute_risk(self):
        return np.clip(self.lightning_density_map / self.max_safe_density, 0, 1)

# ----------------------
# Fusion Module
# ----------------------
class DynamicRiskFusionModule:
    def __init__(self):
        self.base_weights = config["fusion_weights"]

    def fuse(self, risk_maps, uav_status):
        # Start with base weights
        weights = self.base_weights.copy()

        # === Dynamic Weight Adjustments ===
        battery_level = uav_status.get("battery_level", 1.0)
        mission_priority = uav_status.get("mission_priority", "normal")
        distance_to_goal = uav_status.get("distance_to_goal", 0.0)

        # Battery concerns: amplify weather hazards
        if battery_level < 0.3:
            weights["rain"] *= 1.5
            weights["fog"] *= 1.3
            weights["lightning"] *= 1.2

        # High-priority mission: reduce impact of minor risks
        if mission_priority == "high":
            weights["wind"] *= 0.7
            weights["fog"] *= 0.8

        # Long missions: increase concern about lightning/fog
        if distance_to_goal > 200:
            weights["lightning"] *= 1.4
            weights["fog"] *= 1.2

        # Normalize weights (optional but good practice)
        total = sum(weights.values())
        weights = {k: v / total for k, v in weights.items()}

        # === Weighted fusion ===
        total_risk = np.zeros_like(next(iter(risk_maps.values())))
        for key, risk in risk_maps.items():
            total_risk += weights.get(key, 0.0) * risk

        # Logging for debug (can be removed later)
        print("[DynamicFusion] Weights used:", weights)
        print("[DynamicFusion] Mean risk values:", {k: float(np.mean(r)) for k, r in risk_maps.items()})
        print("[DynamicFusion] Fused risk (avg):", float(np.mean(total_risk)))

        return np.clip(total_risk, 0, 1)
