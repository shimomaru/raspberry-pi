import face_recognition
import cv2
import sys

def load_and_encode_image(image_path):
    image = face_recognition.load_image_file(image_path)
    encodings = face_recognition.face_encodings(image)
    if not encodings:
        raise ValueError(f"No face found in image: {image_path}")
    return encodings[0]

def compare_faces(image1_path, image2_path):
    try:
        encoding1 = load_and_encode_image(image1_path)
        encoding2 = load_and_encode_image(image2_path)

        results = face_recognition.compare_faces([encoding1], encoding2)
        distance = face_recognition.face_distance([encoding1], encoding2)

        if results[0]:
            print(f"✅ Match! Distance: {distance[0]:.4f}")
        else:
            print(f"❌ Not the same person. Distance: {distance[0]:.4f}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_faces.py <image1_path> <image2_path>")
        sys.exit(1)

    compare_faces(sys.argv[1], sys.argv[2])

