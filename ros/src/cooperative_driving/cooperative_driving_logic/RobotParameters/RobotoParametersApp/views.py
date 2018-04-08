# -*- coding: utf-8 -*-
from __future__ import unicode_literals
from django.shortcuts import render
from forms import SignUpForm, ContactForm, SlidersForm
import urlparse
from django.http import HttpResponse
from subprocess import call
import os
from django.http import HttpResponseRedirect, JsonResponse
from django.contrib.auth import authenticate ,  login, logout
from django.contrib.auth.decorators import login_required
from django import forms
import os
import subprocess
import string
import json
from django.contrib import messages




# Create your views here.
def home(request):
	# if request.user.is_authenticated():
	# 	return HttpResponseRedirect('/sliders') 
	title = "Welcome! Press one of the following or go to /sliders page to control the robot Parameters"
	invalid = False
	

	form = SignUpForm(request.POST or None)
	
	if form.is_valid():
		username= form.cleaned_data["username"]
		password = form.cleaned_data["password"]
		user = authenticate(username=username,password=password)
		if user is not None:
	  		if user.is_active:
	  			login(request,user)
	  			return HttpResponseRedirect("/sliders/")
	  	else:
	  		invalid = True

	context = {
	"title": title,
	"form": form,
	"invalid": invalid

	 }
	
	return render(request,"home.html",context)


	





def sliderPage(request):
 
 call(["./CopyYaml.sh"])
 f= open("dump.yaml","r")
 contents = f.read()
 maxvelocity = find_between(contents,"max_velocity: ", os.linesep)
 maxturnrate = find_between(contents,"max_turn_rate: ", os.linesep)
 linerowlocation = find_between(contents,"line_row_location: ", os.linesep)
 LF_K_p = find_between(contents,"LF_K_p: ", os.linesep)
 LF_K_i = find_between(contents,"LF_K_i: ", os.linesep)
 LF_K_d = find_between(contents,"LF_K_d: ", os.linesep)
 LF_windup_limit = find_between(contents,"LF_windup_limit: ", os.linesep)
 BD_K_p = find_between(contents,"BD_K_p: ", os.linesep)
 BD_K_i = find_between(contents,"BD_K_i: ", os.linesep)
 BD_K_d = find_between(contents,"BD_K_d: ", os.linesep)
 BD_deadzone = find_between(contents,"BD_deadzone: ", os.linesep)
 BD_windup_limit = find_between(contents,"BD_windup_limit: ", os.linesep)
 target_distance = find_between(contents,"target_distance: ", os.linesep)
 #print maxvelocity, maxturnrate, linerowlocation, LF_K_p, LF_K_i, LF_K_d, LF_windup_limit, BD_K_p, BD_K_i, BD_K_d, BD_deadzone, BD_windup_limit, target_distance






 context = {
 "maxvelocity":maxvelocity,
 "maxturnrate":maxturnrate,
 "linerowlocation":linerowlocation,
 "LF_K_p":LF_K_p,
 "LF_K_i":LF_K_i,
 "LF_K_d":LF_K_d,
 "LF_windup_limit":LF_windup_limit,
 "BD_K_p":BD_K_p,
 "BD_K_i":BD_K_i,
 "BD_K_d":BD_K_d,
 "BD_deadzone":BD_deadzone,
 "BD_windup_limit":BD_windup_limit,
 "target_distance":target_distance

			}

 #print contents

 return render(request,"sliders.html",context)
def ServerPage(request):
	return render(request,"server.html",{})


def sliderPage_vision(request):
 


 return render(request,"vision_sliders.html",{})

def find_between( s, first, last ):
    try:
        start = s.index( first ) + len( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""

def SliderDynamic(request):

	Body= request.body
	ParameterName = find_between(Body,"5B","%5")
	ParameterValue = find_between(Body,"=","&");
	call(["./ChangeLogicParameter.sh",ParameterName,ParameterValue])
	return HttpResponse(" ")


def SliderDynamic_vision(request):
	Body = request.body
	ParameterName = find_between(Body,"5B","%5")
	ParameterValue = find_between(Body,"=","&");
	call(["./ChangeVisionParameter.sh",ParameterName,ParameterValue])
	return HttpResponse(" ")

def StartTheRobot(request):
	
	
	Body = request.body
	print Body
	ID = find_between(Body,"ID%5D=","&csrf")
	Mode= find_between(Body,"5BMode%5D=","&csrf")
	#print Mode
	subprocess.call(['./StartTheRobot.sh',ID,Mode])
	f= open("error.txt","r")
 	contents = f.read()
	return JsonResponse({ 'contents': contents})


def Stop(request):

	call(["./StopTheRobot.sh"])
	return HttpResponse(" ")

def make(request):

	Body = request.body
	Path = find_between(Body,"%2F","&cs")
	filtered_path = string.replace(Path,"%2F","/")
	filtered_path = "/"+filtered_path + "/ros"
	subprocess.call(['./CleanRelease.sh',filtered_path])
	f= open("error.txt","r")
 	contents = f.read()
	return JsonResponse({ 'contents': contents})

def source(request):
	Body = request.body
	Path = find_between(Body,"%2F","&cs")
	filtered_path = string.replace(Path,"%2F","/")
	filtered_path = "/"+filtered_path + "/ros/devel"
	subprocess.call(['./source.sh',filtered_path])
	f= open("error.txt","r")
 	contents = f.read()
	return JsonResponse({ 'contents': contents})


def Restart(request):
	Body = request.body
	print Body
	Oldport = find_between(Body,"oldport%5D=","&JsonNewPort")
	Newport = find_between(Body,"newport%5D=","&csrf")
	call(["./RestartServer.sh",Oldport,Newport])
	return HttpResponse(" ")


