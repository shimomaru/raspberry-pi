import face_recognition
import sys
import os

# Load image paths
known_image_path = "IMG-20250504-WA0002.jpg"
unknown_image_path = "IMG-20250504-WA0002.jpg"

# Check files exist
if not os.path.exists(known_image_path) or not os.path.exists(unknown_image_path):
    print("❌ One or both image files are missing.")
    sys.exit(1)

# Load the known and unknown images
known_image = face_recognition.load_image_file(known_image_path)
unknown_image = face_recognition.load_image_file(unknown_image_path)

# Encode faces (assumes 1 face per image)
known_encodings = face_recognition.face_encodings(known_image)
unknown_encodings = face_recognition.face_encodings(unknown_image)

if not known_encodings:
    print("❌ No face found in known image.")
    sys.exit(1)

if not unknown_encodings:
    print("❌ No face found in unknown image.")
    sys.exit(1)

# Compare the faces
results = face_recognition.compare_faces([known_encodings[0]], unknown_encodings[0])

if results[0]:
    print("✅ Match found: The unknown face matches the known face.")
else:
    print("❌ No match: The unknown face does not match the known face.")

