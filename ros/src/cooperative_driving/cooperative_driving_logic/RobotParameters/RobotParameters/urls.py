"""trydjango URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/1.11/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  url(r'^$', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  url(r'^$', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.conf.urls import url, include
    2. Add a URL to urlpatterns:  url(r'^blog/', include('blog.urls'))
"""
from django.conf import settings
from django.conf.urls import include,url
from django.contrib import admin
from RobotoParametersApp import views
from django.conf.urls.static import static
from django.contrib.auth import views as auth_views
 
urlpatterns = [
    url(r'^sliders/$', views.sliderPage, name ='sliderPage'),
    url(r'^sliders_vision/$', views.sliderPage_vision, name ='slidersPage_vision'),
    url(r'^$', views.home, name ='home'),
    url(r'^admin/', admin.site.urls),
    url(r'^post/', views.SliderDynamic,name = 'SliderDynamic'),
    url(r'^post_vision/', views.SliderDynamic_vision,name = 'SliderDynamic_vision'),
    url(r'^StartTheRobot/', views.StartTheRobot,name = 'StartTheRobot'),
    url(r'^stop/', views.Stop,name = 'Stop'),
    url(r'^source/', views.source,name = 'source'),
    url(r'^Restart/', views.Restart,name = 'Restart'),
    url(r'^Server/', views.ServerPage,name = 'ServerPage')






]


