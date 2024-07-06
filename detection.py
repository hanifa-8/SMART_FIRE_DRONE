import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

interpreter = tflite.Interpreter(model_path="/home/pi/Downloads/fire_detection_model.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

def preprocess_image(image):
    img = cv2.resize(image, (227, 227))
    img = img.astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    return img

cap = cv2.VideoCapture(0)  

C = ['fire', 'non fire']

while True:
    ret, frame = cap.read()
    if not ret:
        break

    preprocessed_frame = preprocess_image(frame)

    interpreter.set_tensor(input_details[0]['index'], preprocessed_frame)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])

    predictions = np.squeeze(output_data)  

    prediction = np.argmax(predictions)

    label = C[prediction]
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.imshow('Fire Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        print("Input details:", input_details)

