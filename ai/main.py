import vision.facerecognition
import time

def main():
    print("Wall-E AI Booting Up...")

    try:
        while True:
            print("AI running")
            vision.facerecognition.FaceRecognition()

            time.sleep(10)
    except KeyboardInterrupt:
        print("\n Shutting Down Wall-E AI")

if __name__ == "__main__":
    main()