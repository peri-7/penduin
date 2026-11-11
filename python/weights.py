import torch
from stable_baselines3 import SAC
import os.path




def export_weights(agent, filename="weights.h"):
    actor = agent.actor 
    
    # sizes taken from the trained network, so they can't disagree with the weights
    hid_dim, obs_dim = actor.latent_pi[0].weight.shape

    with open(filename, 'w') as f:
        f.write("#ifndef WEIGHTS_H_\n#define WEIGHTS_H_\n\n")
        f.write(f"#define HID_DIM {hid_dim}\n#define OBS_DIM {obs_dim}\n\n")

        for name, param in actor.named_parameters():
            # We only want the hidden layers (latent_pi) and the output mean (mu)
            # We explicitly ignore 'log_std' and any critic networks
            if "latent_pi" in name or "mu" in name:
                # Convert PyTorch name (e.g., 'latent_pi.0.weight') to C variable name
                c_name = name.replace('.', '_')
                
                # Flatten the tensor to a 1D numpy array
                flat_data = param.detach().cpu().numpy().flatten()
                
                # Write the C array definition
                f.write(f"float {c_name}[{len(flat_data)}] = {{\n    ")
                
                # Format as float32
                str_values = [f"{val:.8e}f" for val in flat_data]
                f.write(", ".join(str_values))
                
                f.write("\n};\n\n")
            
        f.write("#endif // WEIGHTS_H\n")

if __name__ == "__main__":

    here = os.path.dirname(os.path.abspath(__file__))
    model = f"{here}/../models/sac_QR101_675000_steps.zip"
    filename = f"{here}/../firmware/stepper_motor/weights.h"
    
    agent = SAC.load(model)
    export_weights(agent, filename)