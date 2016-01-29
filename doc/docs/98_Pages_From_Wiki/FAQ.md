# For Windows


# For Linux

### ubuntu seropenport failed permission denied
The tty devices belong to the "dialout" group, I suspect you are not a member of this group and hence are denied access to /dev/ttyS0, so you need to add yourself to that group.

First check if you are a member of that group:

`groups ${USER}`
This will list all the groups you belong to. If you don't belong to the dialout grup then add yourself to it, for example:

`sudo gpasswd --add ${USER} dialout`
You then need to log out and log back in again for it to be effective. Then see if it fixes your problem.

