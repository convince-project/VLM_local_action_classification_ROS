#!/usr/bin/env python3

from rclpy.node import Node 
import rclpy
from identification_interface.srv import ActionType
from identification_interface.msg import ActionInfo
from torchvision import transforms
from transformers import Qwen2_5_VLForConditionalGeneration, BitsAndBytesConfig
from qwen_vl_utils import process_vision_info 
from transformers import AutoProcessor
from identification.mapping import prompts_map, csv_convert_map, actions_classification
from glob import glob
import re
import os
import warnings

class action_classification(Node):
    def __init__(self,node_name,allow_undeclared_parameters=True):
        super().__init__(node_name)

        self.lauch_service_ = self.create_service(ActionType,'action_type',self.return_action)
        self.action_type_publisher_ = self.create_publisher(ActionInfo,'action_info',10)

        model_path = "Qwen/Qwen2.5-VL-7B-Instruct"
        # Create quantization config
        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,  # or load_in_4bit=True for 4-bit quantization
            # Optional: for 4-bit, you can set compute dtype, quant type, etc.
            # bnb_4bit_compute_dtype=torch.bfloat16,
            # bnb_4bit_quant_type="nf4",
            # bnb_4bit_use_double_quant=True,
        )

        # Load quantized model
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            model_path,
            quantization_config=bnb_config,
            device_map="auto",
            # Note: don't force torch_dtype here when using bitsandbytes quantization
        )
        self.processor = AutoProcessor.from_pretrained(model_path)
      
        self.declare_parameter('data_root_path',"")
        self.declare_parameter("id",rclpy.Parameter.Type.INTEGER) 

        self.data_root_path = self.get_parameter("data_root_path").value
        self.id = self.get_parameter("id").value

    def get_data(self):
        video_path = self.data_root_path+"/video"
        if not os.path.isdir(video_path):
            warnings.warn(f"{video_path} does not exist. If this is what you want, ignore this warning.")
            video = None
        else:
            video = glob(video_path+"/*.mp4") 
            if len(video) == 0:
                raise Exception(f"No mp4 file found in {video_path}")
            video = video[0]
        
        csv_path = self.data_root_path+"/csv_files"
        if not os.path.isdir(csv_path):
            warnings.warn(f"{csv_path} does not exist. If this is what you want, ignore this warning.")
            csv_files = None
        else:
            csv_files = glob(csv_path+"/*.csv") 
            if len(csv_files) == 0:
                raise Exception(f"No csv files found in {csv_path}")
            
        images_path = self.data_root_path+"/images"
        if not os.path.isdir(images_path):
            warnings.warn(f"{images_path} does not exist. If this is what you want, ignore this warning.")
            images = None
        else:
            images = glob(images_path+"/*.[pj][pn]g") 
            if len(images) == 0:
                raise Exception(f"No png or jpg files found in {images_path}")
            
        return video,csv_files,images

    def graph_images(self,n,csv_file,folder_name):

        graph_image_name = f"converted_time_series_{n}.png"
        graph_image_name = os.path.join(folder_name,graph_image_name)
        csv_convert_map[self.id].csv_to_image(csv_file,graph_image_name)
    
        return graph_image_name

    def build_base_message(self):
    
        messages = [
            {"role": "system", "content": prompts_map[self.id].SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompts_map[self.id].USER_PROMPT1},
                ]
            },
        ]
        return messages
        
    def inference(self,messages,custom_temp=1.0,max_new_tokens=2048):

        text = self.processor.apply_chat_template(messages,tokenize=False,add_generation_prompt=True)
        image, video_inputs, _ = process_vision_info(messages, image_patch_size=16, return_video_kwargs=True, return_video_metadata=True)
        
        if (image and video_inputs) is not None:
            transfo = transforms.Resize(size=(128,128))

        if video_inputs is not None:
            video_inputs, video_metadatas = zip(*video_inputs)
            video_inputs,video_metadatas = list(video_inputs), list(video_metadatas)
            videos = []
            for video_nb in range(len(video_inputs)):
                video = video_inputs[video_nb][:int(25*video_metadatas[video_nb]['fps'])]
                video = transfo(video)
                videos.append(video)
        else:
            video_metadatas = None 
            videos = None

        if image is not None:
            images = []
            for im in image:
                ima  = transfo(im)
                images.append(ima)
        else: images = None

        inputs = self.processor(text=[text], images=images, videos=videos, padding=True, return_tensors="pt")
        inputs = inputs.to('cuda')

        output_ids = self.model.generate(**inputs, max_new_tokens=max_new_tokens, temperature=custom_temp)
        generated_ids = [output_ids[len(input_ids):] for input_ids, output_ids in zip(inputs.input_ids, output_ids)]
        output_text = self.processor.batch_decode(generated_ids, skip_special_tokens=True, clean_up_tokenization_spaces=True)
        return output_text[0]

    def return_action(self,request,response):
        
        if request.action_type_request:
            
            action_msg = ActionInfo()

            video,csv_files,images = self.get_data()

            messages = self.build_base_message()

            if video!= None:
                messages[1]['content'].append({"video": video})

            folder_name = os.path.join(self.data_root_path,"converted_time_series")
            if not os.path.isdir(folder_name):
                os.mkdir(folder_name)
            elif os.path.isdir(folder_name):
                graph_files = glob(os.path.join(folder_name,"*.[pj][pn]g"))
                graphs = [graph for graph in graph_files if 'converted_time_series' not in graph]
                if len(graphs) != 0:
                    for graph in graphs:
                        messages[1]['content'].append({"image": graph})
                else:
                    warnings.warn("converted_time_series/ folder is present, but empty of non-converted csv files. If this is what you want, ignore this warning.")

            if csv_files!=None:
                for csv_file_index in range(len(csv_files)):
                    graph_image = self.graph_images(csv_file_index,csv_files[csv_file_index],folder_name)
                    messages[1]['content'].append({"image": graph_image})
            
            if images!=None:
                for image in images:
                    messages[1]['content'].append({"image": image})

            
            response1 = self.inference(messages)

            messages.append({"role": "assistant", "content": response1})
            messages.append({
                "role": "user",
                "content": [
                    {"type": "text", "text": prompts_map[self.id].USER_PROMPT2},
                ]
            })
            
            response2 = self.inference(messages)

            match = re.search(r'\d+', response2)
            ano_num = int(match.group()) if match else None
            if ano_num :
                response.action_type = ano_num
                nl_action = actions_classification[self.id][ano_num]
                response.action_name = nl_action
                response.data_folder_name = self.data_root_path

                action_msg.action_id = ano_num
                action_msg.action_name = nl_action
    
                self.action_type_publisher_.publish(action_msg)
            else:
                raise Exception("For some reason no number was given in the response")
            
        else:
            response.action_type = -1
            response.action_name = "Nothing asked"
            response.data_folder_name = self.data_root_path

        return response

def main():

    rclpy.init()

    data_analyse_node = action_classification("identif_node")

    rclpy.spin(data_analyse_node)

    rclpy.shutdown()

if __name__ =="__main__":
    main()
