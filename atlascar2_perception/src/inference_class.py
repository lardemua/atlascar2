import torch
import importlib
import time
import cv2



class Inference:
    def __init__(self, model_path:str, infer_function_name:str, model_loader_name:str, sample_image):
        # Get output reorganizer class name and import
        class_name = 'models.' + infer_function_name
        self.output_function = importlib.import_module(class_name)
        self.model_img_size = self.output_function.model_img_size
        # model_loader_name = self.output_function.model_loader_name
        model_loader = importlib.import_module('models.' + model_loader_name)
        self.original_img_size = (sample_image.shape[0], sample_image.shape[1])
        self.model, self.cuda, self.half, self.engine, self.framework = model_loader.load(self.original_img_size, self.model_img_size, model_path)
        if self.framework == 'torch':
            self.model.eval()
            self.infer = self.infer_torch
        else:
            print('\033[1;31;48m' + "Invalid framework! Aborting..." + '\033[1;37;0m')
            exit()
        # Check if CUDA is available
        if self.cuda:
            if torch.cuda.is_available():
                self.device = "cuda"
            else:
                print('\033[1;31;48m' + "CUDA NOT DETECTED! Aborting..." + '\033[1;37;0m')
                exit()
        else:
            self.device = 'cpu'
        print(f"Using device: {self.device}")
      
    
    def load_image(self, image):
        
        time_a = time.time()
        self.transformed_image, self.original_img_size, self.model_img_size = self.output_function.transforms(image, self.cuda, self.device, self.half)
        time_b = time.time()
        print(f"Carregamento da imagem: {time_b-time_a}")


    def infer_torch(self):
        time_a = time.time()
        with torch.no_grad():
            outputs = self.model(self.transformed_image)
        torch.cuda.synchronize()
        time_b = time.time()
        organized_outputs = self.output_function.output_organizer(outputs, self.original_img_size, self.model_img_size)
        print(f"Tempo de inferência: {time_b-time_a}")
        return organized_outputs
    
