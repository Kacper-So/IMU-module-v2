FROM eclipse-mosquitto

RUN apk update && apk add python3 py3-pip
RUN pip3 install paho-mqtt

COPY main.py /root/main.py

EXPOSE 1883

CMD ["python3", "/root/main.py"]
