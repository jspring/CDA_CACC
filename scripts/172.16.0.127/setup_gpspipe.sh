#!/bin/sh
cd /home/qnxuser/.ssh                                
ssh rsu@172.16.1.120 "mkdir .ssh"                    
scp id_rsa.pub rsu@172.16.1.120:.ssh/authorized_keys 
ssh rsu@172.16.1.120 "gpspipe -r"                    
