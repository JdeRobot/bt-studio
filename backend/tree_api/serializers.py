from rest_framework import serializers


class FileContentSerializer(serializers.Serializer):
    content = serializers.CharField()


class ProjectSerializer(serializers.Serializer):
    id = serializers.CharField()
    name = serializers.CharField()
    creator = serializers.CharField()
    last_modified = serializers.DateTimeField()
