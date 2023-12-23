FROM python:3.6
RUN pip install pymavlink==2.4.0 mavproxy==1.7.1 dronekit dronekit-sitl geopy
COPY entrypoint.sh .
COPY ./scripts/* .
CMD ["/bin/sh", "entrypoint.sh"]