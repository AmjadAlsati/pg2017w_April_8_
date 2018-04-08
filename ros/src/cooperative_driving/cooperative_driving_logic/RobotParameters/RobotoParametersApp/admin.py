# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.contrib import admin
from .models import SignUp
from .forms import SignUpForm

# Register your models here.

class SignUpAdmin(admin.ModelAdmin):
	list_display = ["__unicode__","timestamp","update"]
	form = SignUpForm
	
admin.site.register(SignUp,SignUpAdmin)