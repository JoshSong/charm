@ECHO ON
javac -d ..\ dynamixel_jni.java
javah -d .\ -classpath ..\ dynamixel_jni

@ECHO OFF
PAUSE