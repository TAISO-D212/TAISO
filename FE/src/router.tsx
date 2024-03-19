import { createBrowserRouter } from 'react-router-dom';
import { LoginPage } from './pages/LoginPage';

export const router = createBrowserRouter([
  {path: '/', element: <LoginPage />},
]);
