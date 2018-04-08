#!/bin/bash
python manage.py runserver 0.0.0.0:$2
fuser -k $1/tcp
