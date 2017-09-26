rm -f *.class
javac -Xlint:unchecked *.java
jar -cmf manifest.mf ../Logo.jar *.class
rm *.class
