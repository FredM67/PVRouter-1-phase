name: 🐞 Bug
description: File a bug/issue
title: '<title>'
labels: [bug]
body:
  - type: textarea
    attributes:
      label: Current Behavior
      description: A description of what you're experiencing.
    validations:
      required: false
  - type: textarea
    attributes:
      label: Reproduction / Steps To Reproduce
      description: Link to a repository with steps to reproduce the behavior.
      placeholder: |
        As you can see in this code example/repository
        1. Using this component...
        2. With these properties...
        3. Click '...'
        4. See error...
    validations:
      required: false
  - type: markdown
    attributes:
      value: |
        Bug Reports with a repository with a full reproduction can be answered far quicker, so please consider including as much information as possible to let us help you quicker!
  - type: textarea
    attributes:
      label: Anything else?
      description: |
        Links? References? Anything that will give us more context about the issue you are encountering!
        Tip: You can attach images or log files by clicking this area to highlight it and then dragging files in.
    validations:
      required: false
