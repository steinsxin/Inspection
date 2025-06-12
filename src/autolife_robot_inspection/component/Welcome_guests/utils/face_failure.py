import cv2
import numpy as np
import traceback


class FaceDetector:
    """
    A class to detect faces in images using a pre-trained DNN model.
    """

    def __init__(self, model_file: str, weights_file: str):
        """
        Initialize the FaceDetector class.

        :param model_file: Path to the model configuration file.
        :param weights_file: Path to the model weights file.
        """
        self.net = cv2.dnn.readNetFromCaffe(model_file, weights_file)

    def detect_faces(self, image_data: np.ndarray, output_path: str, confidence_threshold: float = 0.5) -> tuple:
        """
        Detect faces in an image and save the result.

        :param image_data: Input image as a NumPy array.
        :param output_path: Path to save the output image with detected faces.
        :param confidence_threshold: Minimum confidence threshold for face detection.
        :return: A tuple containing the output image path and a boolean indicating if any face was detected.
        """
        # Check if image data is valid
        if image_data is None:
            print("输入的图像数据无效！")
            return None, False

        # Get image dimensions
        (h, w) = image_data.shape[:2]

        # Create a blob from the image
        blob = cv2.dnn.blobFromImage(image_data, 1.0, (300, 300), (104.0, 177.0, 123.0))

        # Set the blob as input to the network
        self.net.setInput(blob)

        # Get the detections
        detections = self.net.forward()

        # Flag to check if any face is detected
        face_detected = False

        # Loop through the detections
        for i in range(detections.shape[2]):
            # Extract the confidence of the current detection
            confidence = detections[0, 0, i, 2]

            # Filter out weak detections
            if confidence > confidence_threshold:
                # Set the flag to True
                face_detected = True

                # Compute the bounding box coordinates
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Draw the bounding box and confidence on the image
                text = "{:.2f}%".format(confidence * 100)
                y = startY - 10 if startY - 10 > 10 else startY + 10
                cv2.rectangle(image_data, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(image_data, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        # Save the output image
        cv2.imwrite(output_path, image_data)

        return output_path, face_detected

    def has_faces(self, image_data: np.ndarray, confidence_threshold: float = 0.5) -> bool:
        """
        Check if an image contains any faces.

        :param image_data: Input image as a NumPy array.
        :param confidence_threshold: Minimum confidence threshold for face detection.
        :return: A boolean indicating if any face was detected.
        """
        # Check if image data is valid
        if image_data is None:
            print("输入的图像数据无效！")
            return False

        # Get image dimensions
        (h, w) = image_data.shape[:2]

        # Create a blob from the image
        blob = cv2.dnn.blobFromImage(image_data, 1.0, (300, 300), (104.0, 177.0, 123.0))

        # Set the blob as input to the network
        self.net.setInput(blob)

        # Get the detections
        detections = self.net.forward()

        # Loop through the detections
        for i in range(detections.shape[2]):
            # Extract the confidence of the current detection
            confidence = detections[0, 0, i, 2]

            # Filter out weak detections
            if confidence > confidence_threshold:
                return True  # Face detected

        return False  # No face detected


def main() -> None:
    """
    Main function to demonstrate face detection using the FaceDetector class.
    """
    model_file = "../model/deploy.prototxt"
    weights_file = "../model/res10_300x300_ssd_iter_140000_fp16.caffemodel"
    face_detector = FaceDetector(model_file, weights_file)

    # Load image data as a NumPy array
    image_path = "../images/01.jpg"
    image_data = cv2.imread(image_path)
    if image_data is None:
        print("无法加载图像，请检查路径是否正确！")
        return

    output_path = "../detected_faces.jpg"

    # Check if the image contains faces
    if face_detector.has_faces(image_data):
        print("图像中存在人脸")
    else:
        print("图像中未检测到人脸")

    # Detect faces and save the result
    result_path, detected = face_detector.detect_faces(image_data, output_path)
    if detected:
        print(f"检测到人脸，结果已保存到 {result_path}")
    else:
        print("未检测到人脸")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during execution: {e}")
        traceback.print_exc()