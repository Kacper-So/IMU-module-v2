FROM eclipse-mosquitto

RUN apt-get update && apt-get install -y python3 python3-pip
RUN pip3 install paho-mqtt

COPY main.py /main.py

EXPOSE 1883

CMD ["python3", "/main.py"]