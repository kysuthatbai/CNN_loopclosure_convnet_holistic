import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from torchvision import models, transforms
from rtabmap_msgs.msg import GlobalDescriptor
from rclpy.qos import qos_profile_sensor_data
from sklearn.decomposition import PCA

class HolisticFeatureExtractor(Node):
    def __init__(self):
        super().__init__('holistic_feature_extractor')
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.model = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
        self.model.eval().to(self.device)
        self.layer = self.model.layer4 
        
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        
        self.gamma = 0.85
        self.buffer = []
        self.pca = None
        self.buffer_size = 30
        self.cached_msgs = []
        
        self.rgb_sub = message_filters.Subscriber(self, Image, '/oakd/rgb/preview/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, '/oakd/rgb/preview/depth', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=100, slop=1.0)
        self.ts.registerCallback(self.sync_callback)
        
        self.desc_pub = self.create_publisher(GlobalDescriptor, '/global_descriptor', 10)

    def get_features(self, x):
        features = []
        def hook(module, input, output):
            features.append(output)
        handle = self.layer.register_forward_hook(hook)
        with torch.no_grad():
            self.model(x)
        handle.remove()
        return features[0]

    def sync_callback(self, rgb_msg, depth_msg):
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        tensor_img = self.transform(cv_rgb).unsqueeze(0).to(self.device)
        
        fm = self.get_features(tensor_img)
        f_flat = torch.flatten(fm, 1).cpu().numpy().flatten().astype(np.float32)
        
        if self.pca is None:
            self.buffer.append(f_flat)
            self.cached_msgs.append((rgb_msg, f_flat))
            
            if len(self.buffer) == self.buffer_size:
                data_matrix = np.array(self.buffer)
                self.pca = PCA(n_components=self.gamma, whiten=True)
                self.pca.fit(data_matrix)
                
                for msg, flat_feat in self.cached_msgs:
                    desc_pca = self.pca.transform(flat_feat.reshape(1, -1)).flatten().astype(np.float32)
                    desc_msg = GlobalDescriptor()
                    desc_msg.header = msg.header 
                    desc_msg.type = 0 
                    desc_msg.info = desc_pca.tobytes()
                    self.desc_pub.publish(desc_msg)
                self.cached_msgs.clear()
            return

        desc_pca = self.pca.transform(f_flat.reshape(1, -1)).flatten().astype(np.float32)
        desc_msg = GlobalDescriptor()
        desc_msg.header = rgb_msg.header 
        desc_msg.type = 0 
        desc_msg.info = desc_pca.tobytes()
        self.desc_pub.publish(desc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HolisticFeatureExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
