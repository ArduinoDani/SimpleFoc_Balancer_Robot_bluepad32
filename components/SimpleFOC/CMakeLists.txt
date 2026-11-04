# Automatikusan összegyűjti az összes .cpp fájlt a src mappából és az összes almappájából
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "src"
    REQUIRES "arduino"
)