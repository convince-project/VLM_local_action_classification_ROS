This project allows you to use a **VLM**, [Qwen2.5-VL-7B-Instruct](https://huggingface.co/Qwen/Qwen2.5-VL-7B-Instruct), deployed **locally**, for **action classification** on **robotics** tasks. The particularity of this package is the **integrability** of the pipeline using **ROS2**. The deployment on a robotic system is straightforward, provided that the used middleware is ROS2.

Also, technically the package can be used for other applications than action classification. This will mainly depend on the used data and the prompts; as long as you keep two user prompts, representing two inferences in chain.

Check the documentation for mode details : https://convince-project.github.io/VLM_local_action_classification_ROS/.

:warning: Tests have only been made on Linux
