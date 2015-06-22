#Used for sending messages to UI. 
#message_type can take the following values:
# 0 - debug	(not sure we need this, but could be used for messages to the UI that might not be displayed to the actual user)
# 1 - info
# 2 - warning
# 3 - error
# 4 - fatal	(not sure we will ever use this)

uint8 message_type
string message
