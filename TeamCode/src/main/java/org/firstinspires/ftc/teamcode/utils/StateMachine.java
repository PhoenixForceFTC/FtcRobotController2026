package org.firstinspires.ftc.teamcode.utils;

//region --- Imports ---
import java.util.List;
//endregion

public class StateMachine<T>
{
    private List<T> _steps;
    private int _currentStepIndex;
    private boolean _hasStarted;

    //region --- Constructor ---
    //--- Constructor to initialize steps and starting index
    public StateMachine(List<T> steps)
    {
        this(steps, 0); //--- Default to starting at index 0
    }

    //--- Constructor with initial index
    public StateMachine(List<T> steps, int initialIndex)
    {
        _steps = steps;
        _currentStepIndex = (initialIndex >= 0 && initialIndex < steps.size()) ? initialIndex : 0;
        _hasStarted = false;
    }
    //endregion

    //region --- Navigation Methods ---
    //--- Move to the next step
    public T next()
    {
        if (!_hasStarted)
        {
            _hasStarted = true; //--- Mark as started
            return getCurrentStep(); //--- Stay on the first step
        }
        if (_currentStepIndex < _steps.size() - 1)
        {
            _currentStepIndex++;
        }
        return getCurrentStep();
    }

    //--- Move to the previous step
    public T previous()
    {
        if (_currentStepIndex > 0)
        {
            _currentStepIndex--;
        }
        return getCurrentStep();
    }

    //--- Reset to the first step
    public void reset()
    {
        _currentStepIndex = 0;
        _hasStarted = false; //--- Reset the start flag
    }

    //--- Reset to a specific step index
    public void resetToStep(int index)
    {
        if (index >= 0 && index < _steps.size())
        {
            _currentStepIndex = index;
        }
    }
    //endregion

    //region --- Retrieval Methods ---
    //--- Get the current step
    public T getCurrentStep()
    {
        return (_steps.isEmpty()) ? null : _steps.get(_currentStepIndex);
    }

    //--- Get the first step
    public T getFirst()
    {
        return (_steps.isEmpty()) ? null : _steps.get(0);
    }

    //--- Get the last step
    public T getLast()
    {
        return (_steps.isEmpty()) ? null : _steps.get(_steps.size() - 1);
    }

    //--- Find the index of a specific step
    public int findStepIndex(T step)
    {
        return _steps.indexOf(step); // Returns -1 if not found
    }

    //--- Jump to a specific step by value
    public boolean jumpToStep(T step)
    {
        int index = findStepIndex(step);
        if (index != -1)
        {
            _currentStepIndex = index;
            return true; // Success
        }
        return false; // Step not found
    }
    //endregion

    //region --- Modification Methods ---
    //--- Add a new step at a specific index
    public void addStep(int index, T step)
    {
        _steps.add(index, step);
    }

    //--- Append a step to the end
    public void addStep(T step)
    {
        _steps.add(step);
    }

    //--- Remove a step by index
    public void removeStep(int index)
    {
        _steps.remove(index);
        if (_currentStepIndex >= _steps.size())
        {
            _currentStepIndex = _steps.size() - 1; // Adjust index if out of bounds
        }
    }

    //--- Replace a step at a specific index
    public void replaceStep(int index, T newStep)
    {
        if (index >= 0 && index < _steps.size())
        {
            _steps.set(index, newStep);
        }
    }
    //endregion

    //region --- Utility Methods ---
    //--- Get the current step index
    public int getCurrentStepIndex()
    {
        return _currentStepIndex;
    }
    //endregion
}
