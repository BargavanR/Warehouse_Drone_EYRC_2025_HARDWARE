import cv2

def open_external_camera(camera_index=3):
    cap = cv2.VideoCapture(camera_index)  # Open the external camera
    
    if not cap.isOpened():
        print("Error: Could not open external camera.")
        return
    
    while True:
        ret, frame = cap.read()
        
        
        if ret:
            height, width, _ = frame.shape  # Get dimensions
            print(f"Image Dimensions: Width = {width}, Height = {height}")
            
        cv2.imshow('Live Video', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    #cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    open_external_camera()