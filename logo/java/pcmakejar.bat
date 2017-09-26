copy Logo.java logo.class
del *.class
javac -Xlint:unchecked *.java
jar -cmf manifest.mf ..\LightLogo.jar *.class
del *.class


