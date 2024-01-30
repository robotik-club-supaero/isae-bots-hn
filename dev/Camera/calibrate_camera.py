import json, argparse, traceback, glob, datetime
import os, sys
import numpy as np
import cv2
from util import loadImage

class Undistort:

    def __init__(self, mtx=None, dist=None, path=None):
        if mtx is None and dist is None and path is None:
            raise ValueError("Either 'mtx' and 'dist' or 'path' must be given")
        if ((mtx is None) != (dist is None)) or ((mtx is None) == (path is None)):
            raise ValueError("'mtx' and 'dist' cannot be given along with 'path'")
        
        if path is not None:
            data = np.load(path)
            self._mtx, self._dist = data["mtx"], data["dist"]
        else:
            self._mtx, self._dist = mtx, dist
        
    def undistort(self, image):
        image = loadImage(image)

        h,  w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self._mtx, self._dist, (w,h), 0, (w,h))
        dst = cv2.undistort(image, self._mtx, self._dist, None, newcameramtx)
        
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst
    
    def __call__(self, image):
        return self.undistort(image)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(prog='Camera calibration')
    parser.add_argument('calibration_file')
    parser.add_argument('--output', '-o', default=".", required=False, help="output directory") 
    parser.add_argument('--draw', '-d', action="store_true", help="draw detected corners on calibration images") 
    parser.add_argument('--epsilon', required=False, type=float, default=1e-3, help="TERM_CRITERIA_EPS in cv2.cornerSubPix") 
    parser.add_argument('--max-count', '-c', required=False, type=int, default=30, help="TERM_CRITERIA_MAX_ITER in cv2.cornerSubPix")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    EPSILON = args.epsilon
    MAX_COUNT = args.max_count

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, MAX_COUNT, EPSILON)

    with open(args.calibration_file) as file:
        value = json.load(file)
        SIZE = (value["cols"], value["rows"])
        SQUARE_SIZE = value["square_size"]

        CHESSBOARD = np.zeros((SIZE[0] * SIZE[1], 3), dtype=np.float32)
        CHESSBOARD[:, :2] = np.mgrid[:SIZE[0], :SIZE[1]].T.reshape(-1, 2)

        IMAGE_SIZE = None

        objpoints = []
        imgpoints = []

        if isinstance(value["images"], list):
            images = value["images"]
        elif isinstance(value["images"], str):
            images = glob.glob(value["images"])
        else:
            raise ValueError("`images` must be either a list of paths or a \"glob\" string")
        
        for path in images:
            image = cv2.imread(path)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            if IMAGE_SIZE is None:
                IMAGE_SIZE = gray.shape[::-1]
            elif IMAGE_SIZE != gray.shape[::-1]:
                raise ValueError("All the images should have the same size")

            ret, corners = cv2.findChessboardCorners(gray, SIZE, None)
            if ret:
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

                objpoints.append(CHESSBOARD)
                imgpoints.append(corners2)

                if args.draw:
                    try:
                        cv2.drawChessboardCorners(image, SIZE, corners2, ret)
                        o_path = os.path.join(args.output, os.path.basename(path))
                        cv2.imwrite(o_path, image)
                        print(f"Drawn corners to `{o_path}`")
                    except:
                        print(f"Warning: Failed to draw corners for image `{path}`", file=sys.stderr)
                        traceback.print_exc()

            else:
                print(f"Warning: Cannot detect corners in image `{path}` (skipped)", file=sys.stderr)
        
        if IMAGE_SIZE is None or len(objpoints) == 0:
            raise ValueError("At least one successful image is required for calibration!")

        print("Corner detection complete. Calibrating camera... This may take a few seconds...")

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, IMAGE_SIZE, None, None)
        
        if args.draw:
            undistort = Undistort(mtx=mtx, dist=dist)
            for path in images:
                image = undistort(path)
                o_path = os.path.join(args.output, "undistorted_" + os.path.basename(path))
                cv2.imwrite(o_path, image)
                print(f"Drawn undistorted image to `{o_path}`")
        
        if not ret:
            print("Warning: Camera calibration failed! Output results may be inaccurate.", file=sys.stderr)
        
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "total error: {}".format(mean_error/len(objpoints)) )
        
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, IMAGE_SIZE, 1, IMAGE_SIZE)

        o_path = datetime.datetime.now().strftime("calibration_%d_%m_%Y_%H_%M_%S.npz")
        o_path = os.path.join(args.output, o_path)

        np.savez(o_path, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs, newcameramtx=newcameramtx, roi=roi)
        print(f"Calibration results saved to {o_path}")