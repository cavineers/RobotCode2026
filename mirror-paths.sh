#!/bin/bash

# Script to mirror PathPlanner paths from right to left side of the field
# This uses the PathMirror utility class to automatically create mirrored path files

cd "$(dirname "$0")" || exit 1

echo "Compiling PathMirror utility..."
javac -cp "src/main/java:$(find ~/.gradle/caches -name 'jackson-databind*.jar' -o -name 'jackson-core*.jar' | head -2 | tr '\n' ':')" \
  -d /tmp/pathcopy \
  src/main/java/frc/lib/PathMirror.java 2>/dev/null

if [ $? -ne 0 ]; then
  echo "Using direct Java method instead..."
  # Fallback: use Java directly with gradle's classpath
  ./gradlew compileJava -q
  
  echo ""
  echo "Mirroring right-side paths to left-side paths..."
  java -cp "build/classes/java/main:build/resources/main" frc.lib.PathMirror
else
  echo ""
  echo "Mirroring right-side paths to left-side paths..."
  java -cp "/tmp/pathcopy:$(find ~/.gradle/caches -name 'jackson*.jar' | tr '\n' ':')" frc.lib.PathMirror
fi

echo ""
echo "✓ Path mirroring complete! Check src/main/deploy/pathplanner/paths for new Left_* paths."
