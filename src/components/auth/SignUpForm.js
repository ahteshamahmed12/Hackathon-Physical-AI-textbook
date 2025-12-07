import React, { useState } from 'react';
import axios from 'axios';

const SignUpForm = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    password2: '',
  });

  const { email, password, password2 } = formData;

  const onChange = (e) =>
    setFormData({ ...formData, [e.target.name]: e.target.value });

  const onSubmit = async (e) => {
    e.preventDefault();
    if (password !== password2) {
      alert('Passwords do not match');
    } else {
      try {
        const config = {
          headers: {
            'Content-Type': 'application/json',
          },
        };

        const body = JSON.stringify({ email, password });

        const res = await axios.post('/api/auth/signup', body, config);
        alert('Sign Up Successful');
      } catch (err) {
        alert(err.response.data.msg || 'Server Error');
      }
    }
  };

  return (
    <form onSubmit={onSubmit}>
      <div>
        <input
          type='email'
          placeholder='Email Address'
          name='email'
          value={email}
          onChange={onChange}
          required
        />
      </div>
      <div>
        <input
          type='password'
          placeholder='Password'
          name='password'
          value={password}
          onChange={onChange}
          minLength='6'
        />
      </div>
      <div>
        <input
          type='password'
          placeholder='Confirm Password'
          name='password2'
          value={password2}
          onChange={onChange}
          minLength='6'
        />
      </div>
      <input type='submit' value='Sign Up' />
    </form>
  );
};

export default SignUpForm;