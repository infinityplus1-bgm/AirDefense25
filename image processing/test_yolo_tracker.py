from ultralytics import YOLO
from datetime import datetime


model = YOLO('best.pt')



results = model.track(source="/home/infinityplusone/Documents/AirDefense25/assets/sample.mp4" , show=True)

# for result in results:
#     for box in result.boxes:
#     # print(type(result))
#     # print(result)
#     # print('-' * 50)
#         print(box.data)