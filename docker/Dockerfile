FROM eclipse-mosquitto

RUN apk update && apk add python3 py3-pip
RUN apk add dpkg-dev build-base sdl2-dev freetype-dev
RUN apk add python3-dev
RUN apk add python3-tkinter
RUN pip3 install paho-mqtt
RUN pip3 install --upgrade pip setuptools
RUN pip3 install matplotlib

COPY ./src/main.py /root/main.py

EXPOSE 1883

# CMD ["python3", "/root/main.py"]
