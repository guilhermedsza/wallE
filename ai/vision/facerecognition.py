import face_recognition
import os, sys
import cv2
import numpy as np
import math
import websocket


def face_confidence(face_distance, face_match_threshold=0.6):
    range = 1.0 - face_match_threshold
    linear_val = (1.0 - face_distance) / (range * 2.0)

    if face_distance > face_match_threshold:
        return str(round(linear_val * 100, 2)) + "%"
    else:
        value = (
            linear_val + ((1.0 - linear_val) * math.pow((linear_val - 0.5) * 2, 0.2))
        ) * 100
        return str(round(value, 2)) + "%"


class FaceRecognition:
    face_locations = []
    face_encodings = []
    face_names = []
    known_face_encodings = []
    known_face_names = []
    process_current_frame = True

    previously_seen_names = set()

    ws = websocket.WebSocket()
    ws.connect("ws://192.168.1.197/ws")

    def __init__(self):
        self.base_path = os.path.dirname(os.path.abspath(__file__))
        self.faces_path = os.path.join(self.base_path, "faces")
        self.encode_faces()

    def recognized_person(self):
        self.ws.send("EYES")

    def encode_faces(self):
        self.known_face_encodings = []
        self.known_face_names = []

        for image in os.listdir(self.faces_path):
            image_path = os.path.join(self.faces_path, image)

            if not os.path.isfile(image_path):
                continue

            print(f"Processing: {image_path}")

            face_image = face_recognition.load_image_file(image_path)
            face_locations = face_recognition.face_locations(face_image)
            face_encodings = face_recognition.face_encodings(face_image, face_locations)

            if len(face_encodings) == 0:
                print(f"Warning: no face found in {image}")
                continue

            if len(face_encodings) > 1:
                print(f"Warning: multiple faces found in {image}, using the first one")

            self.known_face_encodings.append(face_encodings[0])
            self.known_face_names.append(os.path.splitext(image)[0])

        print("Loaded known faces:", self.known_face_names)

    def run_recognition(self):
        video_capture = cv2.VideoCapture(2, cv2.CAP_V4L2)

        if not video_capture.isOpened():
            sys.exit("Video source not found...")

        try:
            while True:
                ret, frame = video_capture.read()

                if not ret or frame is None:
                    print("Warning: Failed to grab frame from camera")
                    continue

                if self.process_current_frame:
                    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

                    self.face_locations = face_recognition.face_locations(
                        rgb_small_frame
                    )
                    self.face_encodings = face_recognition.face_encodings(
                        rgb_small_frame, self.face_locations
                    )

                    self.face_names = []

                    current_seen_names = set()

                    for face_encoding in self.face_encodings:
                        name = "Unknown"
                        confidence = "Unknown"

                        if len(self.known_face_encodings) > 0:
                            matches = face_recognition.compare_faces(
                                self.known_face_encodings, face_encoding
                            )
                            face_distances = face_recognition.face_distance(
                                self.known_face_encodings, face_encoding
                            )
                            best_match_index = np.argmin(face_distances)

                            if matches[best_match_index]:
                                name = self.known_face_names[best_match_index]
                                confidence = face_confidence(
                                    face_distances[best_match_index]
                                )
                                current_seen_names.add(name)

                        self.face_names.append(f"{name} ({confidence})")
                new_names = current_seen_names - self.previously_seen_names

                for name in new_names:
                    self.recognized_person()

                self.previously_seen_names = current_seen_names

                self.process_current_frame = not self.process_current_frame

                for (top, right, bottom, left), name in zip(
                    self.face_locations, self.face_names
                ):
                    top *= 4
                    right *= 4
                    bottom *= 4
                    left *= 4

                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                    cv2.rectangle(
                        frame, (left, bottom - 35), (right, bottom), (0, 0, 255), -1
                    )
                    cv2.putText(
                        frame,
                        name,
                        (left + 6, bottom - 6),
                        cv2.FONT_HERSHEY_DUPLEX,
                        0.8,
                        (255, 255, 255),
                        1,
                    )

                cv2.imshow("Face Recognition", frame)

                if cv2.waitKey(1) == ord("q"):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    fr = FaceRecognition()
    fr.run_recognition()
