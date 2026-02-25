from identification.prompts import prompts_0
from identification.convert_csv_to_image import convert_0

prompts_map ={
    0: prompts_0,
}

csv_convert_map={
    0: convert_0,
}

actions_classification = {
    
    0:{
        1:"I picked a block",
        2:"I picked an object which is not a block.",
        3:"I picked nothing and a human has been detected (one probably intervened in your task).",
        4:"I picked nothing and no human has been detected.",
        5:"Unknown"
        },
}
