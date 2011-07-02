#!/bin/bash
grep --color=always -ir TODO * | grep -v .svn | grep -ir todo
