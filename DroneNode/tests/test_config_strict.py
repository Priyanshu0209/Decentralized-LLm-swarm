import pytest
import yaml

def get_keys(d, prefix=""):
    """Recursively get all keys in a nested dictionary as dot-separated strings."""
    keys = set()
    for k, v in d.items():
        full_key = f"{prefix}.{k}" if prefix else k
        keys.add(full_key)
        if isinstance(v, dict):
            keys.update(get_keys(v, full_key))
    return keys

def test_simulation_and_real_parity():
    """Verify that simulation.yaml and real.yaml define exactly the same structure."""
    with open("config/simulation.yaml", "r") as f:
        sim_config = yaml.safe_load(f)
        
    with open("config/real.yaml", "r") as f:
        real_config = yaml.safe_load(f)
        
    sim_keys = get_keys(sim_config)
    real_keys = get_keys(real_config)
    
    # Ignore the 'mode' key which is intentionally different, and connection.url
    # which might differ, but the KEYS should be the same.
    
    missing_in_real = sim_keys - real_keys
    missing_in_sim = real_keys - sim_keys
    
    assert not missing_in_real, f"Keys in simulation.yaml but missing in real.yaml: {missing_in_real}"
    assert not missing_in_sim, f"Keys in real.yaml but missing in simulation.yaml: {missing_in_sim}"

def test_no_simulation_leak_in_backend():
    import inspect
    from backend_interface import BackendInterface
    from simulation_backend import SimulationBackend
    from real_backend import RealDroneBackend
    
    # All methods in BackendInterface should have the exact same signature in implementations
    base_methods = [f for f in dir(BackendInterface) if callable(getattr(BackendInterface, f)) and not f.startswith('_')]
    
    for method in base_methods:
        if method == "is_armed":
            continue # Builtin or simple
            
        base_sig = inspect.signature(getattr(BackendInterface, method)).replace(return_annotation=inspect.Signature.empty)
        sim_sig = inspect.signature(getattr(SimulationBackend, method)).replace(return_annotation=inspect.Signature.empty)
        real_sig = inspect.signature(getattr(RealDroneBackend, method)).replace(return_annotation=inspect.Signature.empty)
        
        # Verify both backends adhere perfectly to the BackendInterface signature
        assert base_sig == sim_sig, f"{method} signature mismatch in SimulationBackend"
        assert base_sig == real_sig, f"{method} signature mismatch in RealDroneBackend"
