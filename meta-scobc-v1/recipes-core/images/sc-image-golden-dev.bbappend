EXTRA_USERS_PARAMS = " \
    useradd -p \\\$6\\\$F2Aky9vMr5VFWueu\\\$5KDVAzKV8eQmJKZCGbbs4WOKn5Qa9XJopidHLQ7v04HS/PuUQa0H5f7gSKxMrDJXY4mabANpjJ308bTuZ3U7w. petalinux; \
    groupadd -r aie; \
    usermod -a -G aie,audio,input,video petalinux; \
"
