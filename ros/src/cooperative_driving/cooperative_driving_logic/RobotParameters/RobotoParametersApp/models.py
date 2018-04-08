# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models

# Create your models here.

class SignUp(models.Model):
	email = models.EmailField()
	username = models.CharField(max_length=120,blank=True,null=True)
	timestamp = models.DateTimeField(auto_now_add=True,auto_now=False)
	update =  models.DateTimeField(auto_now_add=False,auto_now=True)
	address = models.CharField(max_length=120,blank=False,null=True)
	password = models.CharField(max_length=120,blank= False,null= True)

	def __unicode__(self):
		return self.email