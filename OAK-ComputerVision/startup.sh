echo "-----------NEW RUN------------------" >> /home/tony/depthai-python/TapeDetect/pythonlog.txt

export OPENBLAS_CORETYPE=ARMV8

/home/tony/.virtualenvs/depthAI/bin/python3 /home/tony/depthai-python/TapeDetect/HoughTapeDetect.py >> /home/tony/depthai-python/TapeDetect/pythonlog.txt &

